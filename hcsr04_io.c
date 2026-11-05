/** @file hcsr04_io.c
 * @brief PA2 触发，PB12 双边沿中断记录 DWT 时间戳，信号量唤醒测距任务。
 *
 * 在系统中的角色：HC-SR04 超声波模块的最底层驱动。测距原理：
 * TRIG 发 ≥10µs 高电平脉冲触发测距，模块随后在 ECHO 上输出一个宽度
 * 等于"超声波往返时间"的高电平脉冲；距离 = 脉宽 × 声速 / 2。
 *
 * 计时方案：不用定时器输入捕获，而是用 Cortex-M3 内核的 DWT cycle
 * counter（CYCCNT，随 SystemCoreClock 计数的 32 位自由计数器）。
 * ECHO 配成双边沿 EXTI，中断里只读 CYCCNT 打时间戳并推进状态机，
 * 上升/下降沿的周期差即脉宽。32 位计数在 72MHz 下约 59.6s 回绕一次，
 * 所有差值都用无符号减法，天然容忍回绕（远大于 30ms 的量程需求）。
 * 测量在 RTOS 采集任务中执行，等回波用信号量阻塞而非忙等，CPU 让给
 * 其他任务；本文件即 app_hcsr04 → hcsr04_io 的硬件接口层。
 */
#include "hcsr04_io.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stddef.h>

#define HCSR04_TRIGGER_US (12U)      /* 触发脉宽：手册要求 >10µs，取 12µs 留裕量 */
#define HCSR04_SETTLE_US (2U)        /* 触发前确保 TRIG 为低电平的稳定时间 */
#define HCSR04_WAIT_MS (35U)         /* 信号量等待上限：4m 往返约 23.3ms + 模块处理裕量 */
#define HCSR04_TIMEOUT_US (30000U)   /* 回波脉宽上限 30ms（约 5m），超过视为异常 */
#define HCSR04_MIN_MM (20U)          /* HC-SR04 有效量程近端 2cm */
#define HCSR04_MAX_MM (4000U)        /* HC-SR04 有效量程远端 400cm */
#define HCSR04_CLOCK_DIVISOR (1000000U) /* SystemCoreClock → 每微秒周期数的换算除数 */
#define HCSR04_DELAY_TIMEOUT_MS (5U) /* delay_us 的看门狗上限，DWT 停走时兜底退出 */
#define HCSR04_SOUND_SPEED (343U)    /* 声速 343 m/s（20°C 干燥空气） */
#define HCSR04_DISTANCE_DIVISOR (2000U) /* 距离_mm = 脉宽_us×343/2000：
                                         * 0.343 mm/µs 再除以 2（往返取单程），
                                         * 即 ×343/2000，配合 +1000 四舍五入 */

/* ECHO 边沿状态机：IDLE 空闲 → WAIT_RISE 等上升沿 → WAIT_FALL 等下降沿 → DONE 完成。
 * 状态在 EXTI 中断里推进，测量任务只在关中断的临界区读写，故用 volatile。 */
typedef enum { ECHO_IDLE, ECHO_WAIT_RISE, ECHO_WAIT_FALL, ECHO_DONE } echo_state_t;
static volatile echo_state_t echo_state;
static volatile uint32_t capture_start; /* 触发时刻的 CYCCNT，用于 30ms 总超时判定 */
static volatile uint32_t rise_cycles;   /* 上升沿时间戳（回波开始） */
static volatile uint32_t fall_cycles;   /* 下降沿时间戳（回波结束） */
static uint32_t cycles_per_us;          /* SystemCoreClock/1e6，周期数→微秒换算系数 */
static volatile bool ready;             /* 硬件与信号量初始化完成标志 */
static osSemaphoreId_t echo_done;       /* 下降沿中断释放，测量任务阻塞等待 */

/** @brief 只用于触发信号的短延时；等待回波期间不忙等。
 *
 * 基于 DWT CYCCNT 的忙等延时，精度达单个 CPU 周期，用于产生精确的 12µs
 * 触发脉冲（HAL 的毫秒级延时无法满足微秒要求）。双重退出条件：
 * 正常路径按周期数到达退出；兜底路径在 HAL tick 超过 5ms 时退出并返回
 * false——防御 CYCCNT 意外停走导致死循环（调用方会据此停用测距）。
 * 周期差用无符号减法，兼容 CYCCNT 回绕。 */
static bool hcsr04_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tick = HAL_GetTick();
    while ((uint32_t)(DWT->CYCCNT - start) < us * cycles_per_us) {
        if ((uint32_t)(HAL_GetTick() - tick) >= HCSR04_DELAY_TIMEOUT_MS) { return false; }
    }
    return true;
}

/**
 * @brief 初始化 DWT 周期计数器与同步信号量；任一条件不满足返回 false。
 *
 * 调用上下文：内核已初始化、RTOS 调度器尚未启动（osSemaphoreNew 要求）。
 * 步骤与检查：
 * 1) 计算 cycles_per_us 并做合理性检查（0 或 >1000 说明时钟配置异常）；
 * 2) NOCYCCNT 位为 1 表示该内核/调试配置不支持周期计数器，直接失败；
 * 3) 置 DEMCR.TRCENA 使能 DWT，再置 CTRL.CYCCNTENA 启动计数；
 * 4) 空跑 32 个 NOP 后对比 CYCCNT：若没有变化说明计数器根本没走
 *    （某些调试配置下会被冻结），提前失败而不是留到测量时死等；
 * 5) 创建计数上限 1、初始 0 的二值信号量，供"下降沿中断→测量任务"同步。
 */
bool hcsr04_io_init(void)
{
    uint32_t before;
    uint32_t probe;
    ready = false;
    echo_state = ECHO_IDLE;
    cycles_per_us = SystemCoreClock / HCSR04_CLOCK_DIVISOR;
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    if (cycles_per_us == 0U || cycles_per_us > 1000U ||
        (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U) { return false; }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    before = DWT->CYCCNT;
    for (probe = 0U; probe < 32U; ++probe) { __NOP(); }
    if (DWT->CYCCNT == before) { return false; }
    /* 内核已经初始化，调度器尚未启动，此时允许建立同步对象。 */
    echo_done = osSemaphoreNew(1U, 0U, NULL);
    ready = echo_done != NULL;
    return ready;
}

/** @brief ISR 只记录边沿和释放信号量；不计算距离、不打印日志。
 *
 * PB12 双边沿 EXTI 状态机：先取 CYCCNT 时间戳再读引脚电平，用"电平+当前
 * 状态"共同判定这是哪种边沿（双边沿共用一个中断向量）：
 * - WAIT_RISE 且读到高电平 → 上升沿（回波开始），记 rise_cycles，转 WAIT_FALL；
 * - WAIT_FALL 且读到低电平 → 下降沿（回波结束），记 fall_cycles，转 DONE
 *   并释放信号量唤醒测量任务；释放失败（任务已超时离开）则回 IDLE 防止
 *   残留令牌影响下一次测量；
 * - 状态与电平不匹配（如毛刺、错过边沿）→ 忽略，保持原状态等待。
 * 时间戳必须在中断里第一时间读取，任何计算都推迟到任务上下文，保证
 * 微秒级测量精度。 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    uint32_t stamp;
    GPIO_PinState level;
    if (pin != HCSR04_ECHO_Pin || !ready) { return; }
    stamp = DWT->CYCCNT;
    level = HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin);
    if (echo_state == ECHO_WAIT_RISE && level == GPIO_PIN_SET) {
        rise_cycles = stamp;
        echo_state = ECHO_WAIT_FALL;
    } else if (echo_state == ECHO_WAIT_FALL && level == GPIO_PIN_RESET) {
        fall_cycles = stamp;
        echo_state = ECHO_DONE;
        /* 优先级 5 允许调用 RTOS；CMSIS 封装在中断内使用 FromISR 接口。 */
        if (osSemaphoreRelease(echo_done) != osOK) { echo_state = ECHO_IDLE; }
    }
}

/**
 * @brief 执行一次完整测距：触发脉冲 → 信号量等回波 → 周期差换算距离。
 *
 * @param sample 输出参数，写入脉宽（µs）与距离（mm）；失败时两字段清零。
 * @return HCSR04_OK 成功；各错误码对应可诊断的硬件/时序状态。
 * 调用上下文：RTOS 采集任务（需调度器运行中），周期约 500ms。
 *
 * 流程与同步设计：
 * 1) 前置检查：ECHO 静息必须为低，若已为高说明上次回波未结束或接线异常
 *    （ECHO_HIGH），此时触发必然测错，直接拒绝；
 * 2) 排空信号量：上次超时返回后，迟到的下降沿可能已释放令牌，先非阻塞
 *    取空，防止本次等待立即假成功；
 * 3) osKernelLock + 关中断临界区：触发序列（低 2µs → 高 12µs → 拉低）
 *    期间禁止调度和中断，保证脉冲宽度精确；同时清 EXTI 挂起位、记
 *    capture_start、置 WAIT_RISE——清挂起与置状态必须原子，否则残留的
 *    挂起中断会在状态就绪前触发；
 * 4) 恢复调度后阻塞等信号量（35ms 上限），期间 CPU 让给其他任务；
 * 5) 结果在关中断临界区一次性拷出并复位状态机，与 ISR 无竞争；
 * 6) 分类：未 DONE 时，WAIT_RISE=从未见上升沿（NO_ECHO，查接线/供电），
 *    WAIT_FALL=上升沿后迟迟不落（ECHO_TIMEOUT，目标过远或 ECHO 卡高）；
 *    DONE 后再按 fall-start 校验总时长不超 30ms，防御异常时间戳；
 * 7) 距离换算：mm = 脉宽µs × 343 / 2000（声速 343m/s，往返取单程，
 *    分子加 1000 即除数一半实现四舍五入），最后做 2..400cm 量程检查。
 */
hcsr04_result_t hcsr04_io_measure(hcsr04_sample_t *sample)
{
    uint32_t mask;
    uint32_t rise, fall, start;
    echo_state_t captured;
    int32_t lock;
    bool trigger_ok;
    osStatus_t waited;
    if (sample == NULL) { return HCSR04_INVALID_ARGUMENT; }
    sample->pulse_us = sample->distance_mm = 0U;
    if (!ready || osKernelGetState() != osKernelRunning) { return HCSR04_NOT_READY; }
    if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET) {
        return HCSR04_ECHO_HIGH;
    }
    /* 清除上次超时与结束边沿竞态中可能留下的令牌。 */
    while (osSemaphoreAcquire(echo_done, 0U) == osOK) { }
    lock = osKernelLock();
    if (lock < 0) { return HCSR04_NOT_READY; }
    mask = __get_PRIMASK();
    __disable_irq();
    __HAL_GPIO_EXTI_CLEAR_IT(HCSR04_ECHO_Pin);
    capture_start = DWT->CYCCNT;
    echo_state = ECHO_WAIT_RISE;
    __set_PRIMASK(mask);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    trigger_ok = hcsr04_delay_us(HCSR04_SETTLE_US);
    if (trigger_ok) {
        HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
        trigger_ok = hcsr04_delay_us(HCSR04_TRIGGER_US);
    }
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    if (osKernelRestoreLock(lock) < 0) { return HCSR04_NOT_READY; }
    if (!trigger_ok) { echo_state = ECHO_IDLE; ready = false; return HCSR04_NOT_READY; }
    /* 回波等待进入阻塞态，其余任务可以调度；总等待有超时上限。 */
    waited = osSemaphoreAcquire(echo_done, (HCSR04_WAIT_MS * osKernelGetTickFreq() + 999U) / 1000U);
    mask = __get_PRIMASK();
    __disable_irq();
    captured = echo_state;
    rise = rise_cycles;
    fall = fall_cycles;
    start = capture_start;
    echo_state = ECHO_IDLE;
    __set_PRIMASK(mask);
    if (captured != ECHO_DONE) {
        if (waited != osErrorTimeout && waited != osOK) { return HCSR04_NOT_READY; }
        return captured == ECHO_WAIT_RISE ? HCSR04_NO_ECHO : HCSR04_ECHO_TIMEOUT;
    }
    if ((uint32_t)(fall - start) > HCSR04_TIMEOUT_US * cycles_per_us) {
        return HCSR04_ECHO_TIMEOUT;
    }
    sample->pulse_us = (uint32_t)(fall - rise) / cycles_per_us;
    sample->distance_mm = (sample->pulse_us * HCSR04_SOUND_SPEED + HCSR04_DISTANCE_DIVISOR / 2U) /
                          HCSR04_DISTANCE_DIVISOR;
    return sample->distance_mm >= HCSR04_MIN_MM && sample->distance_mm <= HCSR04_MAX_MM ?
           HCSR04_OK : HCSR04_OUT_OF_RANGE;
}
