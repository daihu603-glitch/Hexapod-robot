/**
 * @file app_servo.c
 * @brief 舵机命令层：把全局舵机编号和目标角度转换成对应 PCA9685 通道的脉宽。
 */
/* 【中文详解】六足机器人 18 路舵机的软件抽象层：
 * 硬件拓扑：两块 16 通道 PCA9685 PWM 驱动板，分别挂在 I2C1 和 I2C2 上。
 * 因为位于两条独立总线，两块板都可以使用出厂默认七位地址 0x40，无需跳线改址。
 * 编号规则：全局逻辑编号 0~8 -> I2C1 板（devices[0]），9~17 -> I2C2 板（devices[1]）。
 * 通道映射：PCA9685 每 4 通道共用一个推挽输出组，硬件上每四路只焊了前三路，
 * 所以每块板只用 {0,1,2, 4,5,6, 8,9,10} 九个通道（CH3/CH7/CH11~CH15 空置）。
 * 角度-脉宽换算：45°=1000us，90°=1500us，135°=2000us（标准 50Hz 舵机标称值）。
 * 调用上下文：仅 servo 任务调用（I2C 独占免锁）；app_servo_init 例外，
 * 由 network 任务在初始化阶段调用，此时 servo 任务尚未放行。 */
#include "app_servo.h"
#include "pca9685.h"
#include <stdio.h>
#include <string.h>

#define SERVO_PER_BOARD 9U    /* 每块 PCA9685 实际接 9 路舵机（见通道映射表） */
#define SERVO_COUNT 18U       /* 全局舵机总数 = 2 块 × 9 路，对应六足 3 关节 × 6 腿 */
#define SERVO_MIN_DEG 45U     /* 角度下限：机械结构限位，防止舵机堵转/拉扯连杆 */
#define SERVO_MAX_DEG 135U    /* 角度上限：同上；45~135 恰好覆盖 1000~2000us 标准脉宽 */
/* 每四路只用前三路；每块接九个舵机，CH3、CH7、CH11～CH15 留空。
 * 手机编号保持 0～17，角度命令与单路关闭均通过此表选择实际 PWM 通道。 */
/* 表下标 = id % 9（板内序号），表值 = PCA9685 物理通道号。 */
static const uint8_t servo_channels[SERVO_PER_BOARD] = {0U, 1U, 2U, 4U, 5U, 6U, 8U, 9U, 10U};
/* 两块模块使用独立 I2C 总线，因此均可保留默认七位地址 0x40。 */
/* 字段：{I2C 总线号(1/2 -> hi2c1/hi2c2), 七位地址, ready 标志}。
 * ready 由首次成功 pca9685_init 置位，实现惰性初始化：
 * 未接的板不会在启动时反复探测拖慢初始化。 */
static pca9685_t devices[2] = {{0U, 0x40U, 0U}, {1U, 0x40U, 0U}};

/**
 * @brief 启动时尝试关闭两块模块的输出；模块缺失不阻塞 Wi-Fi 启动，也不自动指定角度。
 * @note 调用上下文：network 任务的启动序列（app_rtos_network_run），
 *       在 RTOS_INIT_DONE 置位之前，此时 servo 任务还在等待，无 I2C 竞争。
 *       场景：STM32 单独复位（如看门狗/调试器）时 PCA9685 未断电，
 *       其内部寄存器仍保留旧 PWM 输出，舵机会保持上电前姿态甚至持续受力；
 *       因此先尝试全通道关断，失败（板未接）则忽略，不影响后续启动。
 */
void app_servo_init(void)
{
    uint8_t i;
    /* 仅 STM32 复位时，PCA9685 可能仍保留旧 PWM，先尝试关闭。
     * 未接模块不阻塞网络启动；此处不自动控制舵机转到某个角度。 */
    for (i = 0U; i < 2U; ++i) {
        devices[i].ready = 0U;   /* 清零惰性初始化标志，首条命令时才真正配置 */
        (void)pca9685_all_off(&devices[i]);   /* 忽略返回值：缺板是允许的运行状态 */
    }
}

/**
 * @brief 读取最多三位无符号十进制数；成功时推进字符指针并返回 1，格式错误返回 0。
 * @param cursor 输入/输出：解析游标，成功时指向数字之后的第一个字符。
 * @param value  输出：解析结果（最大 999，覆盖角度 135 与编号 17）。
 * @note 严格模式：至少一位数字、至多三位，不接受 '+'/'-' 前缀；
 *       调用方负责检查游标处的分隔符，杜绝 "90x" 被当成 90 执行。
 */
static int number(const char **cursor, uint16_t *value)
{
    uint16_t result = 0U;
    uint8_t digits = 0U;
    const char *p = *cursor;
    while (*p >= '0' && *p <= '9') {
        if (++digits > 3U) { return 0; }   /* 第四位数字即非法，防溢出与误输入 */
        result = (uint16_t)(result * 10U + (uint16_t)(*p++ - '0'));
    }
    if (!digits) { return 0; }             /* 没有数字：格式错误 */
    *cursor = p;
    *value = result;
    return 1;
}

/**
 * @brief 处理 <编号> <角度|off>、off 和 status；回复写入 reply，capacity 为缓冲区字节数。
 * 编号 0～8 使用 I2C1，编号 9～17 使用 I2C2；每块依次映射到 CH0、1、2、4、5、6、8、9、10。
 * 角度仅允许 45～135；首次使用某块模块时才配置其 PWM 参数。
 * @param payload  命令载荷（已剥掉 "servo " 前缀），来自 UART4 或 MQTT。
 * @param reply    输出：执行结果文本（"ok ..." 或 "error ..."），供回包/日志使用。
 * @param capacity reply 缓冲字节数，所有输出经 snprintf 截断保护。
 * @note 仅 servo 任务调用（RTOS 队列串行化），I2C 访问无需加锁。
 *       本函数保留独立完整校验：即使上游 app_esp_link 已预校验，
 *       MQTT/控制台路径也必经此处把关（纵深防御，防止非法角度驱动舵机）。
 */
void app_servo_command(const char *payload, char *reply, size_t capacity)
{
    uint16_t id, angle, pulse;
    uint8_t board, channel;
    pca9685_result_t result, second;
    const char *p = payload;
    if (!payload || !reply || !capacity) { return; }
    if (strcmp(payload, "status") == 0) {
        /* 只报告两块板的 ready 配置标志，不发起 I2C 通信（查询零副作用） */
        (void)snprintf(reply, capacity, "servo configured=%u,%u; ids=0..17; range=45..135",
                       devices[0].ready, devices[1].ready);
        return;
    }
    if (strcmp(payload, "off") == 0) {
        /* 全局关断：撤销两块板所有通道的 PWM（输出置高阻），用于急停/安全断电；
         * 两块板独立尝试并分别报告，一块失败不影响另一块。 */
        result = pca9685_all_off(&devices[0]);
        second = pca9685_all_off(&devices[1]);
        (void)snprintf(reply, capacity, "servo off: I2C1=%s I2C2=%s",
                       result == PCA9685_OK ? "ok" : "error", second == PCA9685_OK ? "ok" : "error");
        return;
    }
    /* "<id> <angle|off>"：编号 0~17，其后恰好一个空格；任何格式偏差走 invalid */
    if (!number(&p, &id) || id >= SERVO_COUNT || *p++ != ' ') { goto invalid; }
    if (strcmp(p, "off") == 0) { pulse = 0U; }   /* 单路关断：脉宽 0 = 撤销该通道 PWM */
    else {
        /* 角度必须完整解析到串尾且在 45~135 机械安全区间内 */
        if (!number(&p, &angle) || *p || angle < SERVO_MIN_DEG || angle > SERVO_MAX_DEG) { goto invalid; }
        /* 使用保守的标称映射：45 度对应 1000、90 度对应 1500、135 度对应 2000 微秒。
         * 实际机械角度取决于舵机型号与安装方式，需要单独校准。 */
        /* 换算：pulse = 1000 + (angle-45) × 1000/90，+45 实现整数四舍五入。 */
        pulse = (uint16_t)(1000U + ((uint32_t)(angle - SERVO_MIN_DEG) * 1000U + 45U) / 90U);
    }
    /* 全局编号 -> (板号, 物理通道)：整除 9 选板（0=I2C1，1=I2C2），
     * 取余 9 查 servo_channels 映射表得到 PCA9685 通道号。 */
    board = (uint8_t)(id / SERVO_PER_BOARD);
    channel = servo_channels[id % SERVO_PER_BOARD];
    result = PCA9685_OK;
    /* 惰性初始化：首次使用某块板时才写 PCA9685 配置寄存器（预分频/模式），
     * 避免启动时探测不存在的板。 */
    if (!devices[board].ready) { result = pca9685_init(&devices[board]); }
    if (result == PCA9685_OK) { result = pca9685_set_pulse(&devices[board], channel, pulse); }
    if (result != PCA9685_OK) {
        /* 错误文本以 "error" 开头：app_esp_link_result 据此把回包状态映射为 ERR；
         * 附带总线号与错误码便于排查接线/供电/地址问题。 */
        (void)snprintf(reply, capacity, "error servo %u: I2C%u code=%u (check wiring/power/address)",
                       (unsigned)id, (unsigned)(board + 1U), (unsigned)result);
        return;
    }
    (void)snprintf(reply, capacity, "ok servo %u pulse=%u us", (unsigned)id, (unsigned)pulse);
    return;
invalid:
    (void)snprintf(reply, capacity, "error servo: use <0..17> <45..135|off>, off, status");
}

/* Servo task only. PCA9685 writes are sequential, not electrically simultaneous. */
/**
 * @brief 一次性输出完整 18 关节角度帧（步态插补的单帧下发接口）。
 * @param degrees 18 个目标角度（浮点，度），下标即全局舵机编号 0~17。
 * @return 1 全部 18 路写入成功；0 校验失败或任一 I2C 写失败（已尽力关断）。
 * @note 仅 servo 任务调用（app_motion 步态状态机每 20ms 下发一帧）。
 *       安全策略："先整帧校验、后统一写入"——任何一个角度越界都拒绝整帧，
 *       避免机器人出现半帧新姿态半帧旧姿态的危险扭曲。
 *       写入是按通道顺序进行的（I2C 逐次事务），并非电气上的同时动作，
 *       但 20ms 帧周期内完成，视觉上近似同步。
 */
int app_servo_frame(const float degrees[18])
{
    unsigned i;
    uint16_t pulse;
    if (degrees == NULL) { return 0; }
    /* 第一遍：完整校验所有 18 个角度都在 45~135 内，越界则整帧拒绝，
     * 此时尚未触碰任何 I2C 设备，舵机保持上一帧姿态。 */
    for(i=0;i<SERVO_COUNT;++i) {
        if (!(degrees[i]>=SERVO_MIN_DEG && degrees[i]<=SERVO_MAX_DEG)) { return 0; }
    }
    /* 两块板都需就绪（惰性初始化）；任一块配置失败则整帧放弃 */
    for(i=0;i<2U;++i) {
        if (!devices[i].ready && pca9685_init(&devices[i])!=PCA9685_OK) { goto failed; }
    }
    /* 第二遍：逐路换算脉宽并写入。浮点换算 +0.5f 实现四舍五入，
     * 与 app_servo_command 的整数算法 (+45)/90 等价。
     * i/SERVO_PER_BOARD 选板，i%SERVO_PER_BOARD 查通道映射表。 */
    for(i=0;i<SERVO_COUNT;++i) {
        pulse=(uint16_t)(1000.0f+(degrees[i]-SERVO_MIN_DEG)*1000.0f/90.0f+0.5f);
        if (pca9685_set_pulse(&devices[i/SERVO_PER_BOARD],servo_channels[i%SERVO_PER_BOARD],pulse)!=PCA9685_OK) {
            goto failed;
        }
    }
    return 1;
failed:
    /* Best effort only: failed I2C cannot guarantee PWM removal. */
    /* 中途失败的安全兜底：立即尝试关断两块板全部输出，让舵机卸力，
     * 防止帧写到一半时剩余通道保持旧 PWM 造成姿态扭曲持续受力。
     * 若 I2C 本身已故障，关断也可能失败，只能尽力而为。 */
    (void)pca9685_all_off(&devices[0]);
    (void)pca9685_all_off(&devices[1]);
    return 0;
}
