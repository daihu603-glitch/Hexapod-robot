/** @file app_rtos.c
 * @brief 四任务调度：通信、舵机、距离采集、串口日志与健康监测。
 * 业务线程只使用 CMSIS-RTOS2；FreeRTOS 原生接口仅用于内核故障钩子。
 */
/* 【中文详解】任务架构与数据流：
 *   network（defaultTask，1ms 周期）：MQTT(Wi-Fi/USART3) + ESP32(UART4) 协议处理，
 *     并完成全部模块的一次性初始化，最后置 RTOS_INIT_DONE 事件放行其他任务。
 *   servo（20ms 周期）：从命令队列取命令执行（I2C 独占），驱动 18 路舵机/步态，
 *     结果写入结果队列；20ms 约等于 50Hz 舵机 PWM 帧周期，也限定了步态插补节拍。
 *   distance（500ms 周期）：HC-SR04 测距，结果放入单槽邮箱队列（旧值自动淘汰）。
 *   log_health（20ms 轮询日志队列）：独占调试串口 USART1 发送；每 5s 检查
 *     三个心跳位（RTOS_BEAT_ALL=7）与各任务栈余量，异常打印 WARNING。
 * 同步原则：任务间零共享缓冲区，一切数据经队列按值复制传递；
 * 启动顺序用事件标志 RTOS_INIT_DONE 控制，心跳用事件标志位实现。 */
#include "app_rtos.h"
#include "app_mqtt.h"
#include "app_servo.h"
#include "app_motion.h"
#include "app_hcsr04.h"
#include "app_esp_link.h"
#include "board_io.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define RTOS_LOG_BYTES (224U)      /* 单条日志上限：与 board_io_log 单次 220 字节限制匹配，留余量 */
#define RTOS_COMMAND_BYTES (96U)   /* 舵机/步态命令文本上限，覆盖最长 "gait ..." 命令 */
#define RTOS_RESULT_BYTES (80U)    /* 执行结果文本上限，与 app_rtos_servo_result 要求的 80 字节一致 */
#define RTOS_QUEUE_DEPTH (8U)      /* 三个多槽队列深度：8 条缓冲足以吸收突发命令，满时拒绝而非阻塞 */
#define RTOS_NET_MS (1U)           /* 通信任务轮询周期 1ms：保证 UART 字节及时入协议缓冲，低延迟 */
#define RTOS_SENSOR_MS (500U)      /* 测距周期 500ms：HC-SR04 两次测量需间隔以等回波消散，且避障无需更快 */
#define RTOS_SERVO_WAIT_MS (20U)   /* 舵机任务周期 20ms = 50Hz，与舵机 PWM 帧率同数量级，步态动作平滑 */
#define RTOS_HEALTH_MS (5000U)     /* 健康报告周期 5s：足够长以免刷屏，足够短以及时发现任务停摆 */
#define RTOS_STACK_WARNING_BYTES (256U) /* 栈剩余低于 256 字节即告警，预留安全裕度 */
/* 心跳位掩码：每个任务每轮循环置一位，log_health 每 5s 一次性读取并清零；
 * 三位全为 1（=RTOS_BEAT_ALL）表示三个业务任务都在跑。 */
#define RTOS_BEAT_NET (1U)         /* bit0：network 任务心跳 */
#define RTOS_BEAT_SENSOR (2U)      /* bit1：distance 任务心跳 */
#define RTOS_BEAT_SERVO (4U)       /* bit2：servo 任务心跳 */
#define RTOS_BEAT_ALL (7U)         /* 三个心跳位的合集，健康检查用 */
#define RTOS_LOG_OVERFLOW (8U)     /* bit3：日志队列曾满（丢弃过日志）的粘滞标志 */
#define RTOS_RESULT_OVERFLOW (16U) /* bit4：结果队列曾满的粘滞标志 */
#define RTOS_INIT_DONE (32U)       /* bit5：network 任务已完成全部模块初始化，其他任务可开工 */

/* 队列消息类型：全部按值复制入队，生产/消费双方不共享缓冲区，
 * 因此无需互斥锁；结构体大小即队列 item 大小。 */
typedef struct { char text[RTOS_LOG_BYTES]; } log_message_t;
typedef struct { char text[RTOS_COMMAND_BYTES]; uint32_t request_id; } command_message_t;
typedef struct { char text[RTOS_RESULT_BYTES]; uint32_t request_id; } result_message_t;

/* 对象句柄仅在内核启动前写入，任务之间的运行期数据全部通过队列/事件传递。 */
static osMessageQueueId_t log_queue;      /* 各任务 -> log_health：待输出日志 */
static osMessageQueueId_t command_queue;  /* network -> servo：舵机/步态命令（含事务号） */
static osMessageQueueId_t result_queue;   /* servo -> network：执行结果（含事务号） */
static osMessageQueueId_t distance_queue; /* sensor -> network：最新测距，深度 1 的邮箱 */
static osEventFlagsId_t health_events;    /* 心跳位 + 溢出标志 + 初始化完成位 */
static osThreadId_t network_handle;       /* 以下四个句柄仅供 log_health 查询栈余量 */
static osThreadId_t sensor_handle;
static osThreadId_t servo_handle;
static osThreadId_t service_handle;

/* 任务属性：三个自建任务栈均 2048 字节。优先级设计：
 * servo(AboveNormal1) > distance(AboveNormal) > network(Normal，CubeMX 默认)
 * > log_health(BelowNormal)。舵机最高以保证 20ms 控制节拍不被网络/日志抢占拖延；
 * 日志最低，串口输出慢也不影响控制与通信。 */
static const osThreadAttr_t sensor_attr = {
    .name = "distance", .stack_size = 2048U, .priority = osPriorityAboveNormal
};
static const osThreadAttr_t servo_attr = {
    .name = "servo", .stack_size = 2048U, .priority = osPriorityAboveNormal1
};
static const osThreadAttr_t service_attr = {
    .name = "log_health", .stack_size = 2048U, .priority = osPriorityBelowNormal
};

/** @brief CMSIS 参数使用 tick，本工程强制 1000Hz，所有时长明确从毫秒转换。 */
/* +999 实现向上取整：不足 1 个 tick 的时长也至少等 1 tick，避免变成 0（忙等）。
 * app_rtos_init 已断言 tick 频率为 1000Hz，故等价于 ms 数值本身。 */
static uint32_t rtos_ticks(uint32_t ms)
{
    return (ms * osKernelGetTickFreq() + 999U) / 1000U;
}

/** @brief 周期目标发生超期时跳过错过的周期，避免连续空转追赶。 */
/* 绝对周期调度：*next 记录下一次应唤醒的 tick。若本轮执行超时导致
 * *next 已落后于当前时刻，直接重置为 now+period（丢弃错过的周期），
 * 防止任务连续零延迟运行去"补帧"，造成 CPU 突发满载与节拍抖动。
 * @param next 输入/输出：下一次唤醒时刻（tick）。
 * @param ms   期望周期（毫秒）。 */
static void rtos_wait_period(uint32_t *next, uint32_t ms)
{
    uint32_t period = rtos_ticks(ms);
    uint32_t now = osKernelGetTickCount();
    *next += period;
    /* 有符号差值比较：兼容 tick 计数回绕（约 49.7 天） */
    if ((int32_t)(*next - now) <= 0) { *next = now + period; }
    if (osDelayUntil(*next) != osOK) { Error_Handler(); }
}

/** @brief 置健康事件标志位；失败说明事件对象被破坏，属致命错误直接停机。 */
static void rtos_event(uint32_t flag)
{
    if ((osEventFlagsSet(health_events, flag) & osFlagsError) != 0U) { Error_Handler(); }
}

/** @brief 查询调度器是否已运行。board_io_log 据此选择直写串口（启动前）
 *  还是入日志队列（运行中），避免任务未创建时向空队列投递。 */
bool app_rtos_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 提交一条日志到日志队列（任意任务可调用的安全日志入口）。
 * @param text 以 NUL 结尾的日志文本；超过 223 字节被静默截断。
 * @note 非阻塞（超时 0）：队列满时丢弃本条并置 RTOS_LOG_OVERFLOW 粘滞位，
 *       由 log_health 在健康报告中体现。日志绝不允许阻塞业务任务。
 */
void app_rtos_log_submit(const char *text)
{
    log_message_t message = {{0}};
    size_t length;
    if (text == NULL || log_queue == NULL) { return; }   /* 队列创建前（启动早期）静默忽略 */
    length = strlen(text);
    if (length >= sizeof(message.text)) { length = sizeof(message.text) - 1U; }
    memcpy(message.text, text, length);   /* 按值复制，调用方缓冲可立即复用 */
    if (osMessageQueuePut(log_queue, &message, 0U, 0U) != osOK) {
        rtos_event(RTOS_LOG_OVERFLOW);    /* 满则丢弃并记账，不重试不阻塞 */
    }
}

/** @brief 无事务号提交舵机命令（MQTT/控制台来源）；见 app_rtos_servo_submit_link。 */
bool app_rtos_servo_submit(const char *payload)
{
    return app_rtos_servo_submit_link(payload, 0U);   /* request_id=0 表示无需回包 */
}

/**
 * @brief 把命令文本按值复制入命令队列，供 servo 任务异步执行。
 * @param payload    命令载荷，如 "0 90"、"off"、"status"、"gait walk"。
 * @param request_id UART4 事务号；0 表示无编号（结果不回报 ESP32）。
 * @return true 已入队；false 队列满/参数非法/超长（调用方自行回 ERR）。
 * @note 仅 network 任务调用。非阻塞：拥塞时快速失败，防止通信任务被
 *       舵机执行拖住；"成功"只代表排队，不代表动作已执行。
 */
bool app_rtos_servo_submit_link(const char *payload, uint32_t request_id)
{
    command_message_t message = {{0}, 0U};
    size_t length;
    if (payload == NULL || command_queue == NULL) { return false; }
    length = strlen(payload);
    if (length == 0U || length >= sizeof(message.text)) { return false; }   /* 空命令或超 95 字节拒绝 */
    memcpy(message.text, payload, length);
    message.request_id = request_id;   /* 事务号随命令穿过 servo 任务再回到 network */
    return osMessageQueuePut(command_queue, &message, 0U, 0U) == osOK;
}

/**
 * @brief 从结果队列取出一条执行结果（仅 network 任务调用）。
 * @param out      输出缓冲，接收固定 80 字节的结果文本。
 * @param capacity out 的字节数，必须 >= RTOS_RESULT_BYTES。
 * @return true 取到一条；false 队列空或参数非法。
 * @note 若结果携带非零事务号，就地调用 app_esp_link_result 生成
 *       "@id OK/ERR ..." 回包，完成 UART4 事务闭环。
 */
bool app_rtos_servo_result(char *out, size_t capacity)
{
    result_message_t message;
    if (out == NULL || capacity < sizeof(message.text) || result_queue == NULL) { return false; }
    if (osMessageQueueGet(result_queue, &message, NULL, 0U) != osOK) { return false; }
    if (message.request_id != 0U) { app_esp_link_result(message.request_id, message.text); }
    memcpy(out, message.text, sizeof(message.text));
    return true;
}

/**
 * @brief 投递最新测距结果到单槽邮箱队列（仅 distance 任务调用）。
 * @param message 测距消息（结果 + 采样细节 + 测量时刻），按值复制。
 * @note 队列深度为 1：消费者只关心"最新值"。满时先取走旧样本再放入
 *       新样本（覆盖语义），既不阻塞采集节拍，也不让陈旧数据滞留。
 */
void app_rtos_distance_put(const app_distance_message_t *message)
{
    app_distance_message_t old;
    if (message == NULL) { return; }
    if (osMessageQueuePut(distance_queue, message, 0U, 0U) != osOK) {
        /* 单一生产者；消费者即使先取走旧值，也不影响随后写入新值。 */
        (void)osMessageQueueGet(distance_queue, &old, NULL, 0U);
        /* 腾位后仍失败说明内核对象异常，属致命错误 */
        if (osMessageQueuePut(distance_queue, message, 0U, 0U) != osOK) { Error_Handler(); }
    }
}

/** @brief network 任务非阻塞取出最新测距；无新样本返回 false（不等待）。 */
bool app_rtos_distance_get(app_distance_message_t *message)
{
    return message != NULL && osMessageQueueGet(distance_queue, message, NULL, 0U) == osOK;
}

/**
 * @brief 距离采集任务：500ms 周期驱动 HC-SR04 测距并投递最新结果。
 * @param argument 未使用。
 * @note 测距结果的格式化与队列投递由 app_hcsr04_process 内部完成；
 *       本任务只负责节拍与心跳。
 */
static void rtos_sensor_task(void *argument)
{
    uint32_t next;
    (void)argument;
    /* 通信任务完成一次性初始化后才采集，避免访问未初始化的信号量。 */
    /* osFlagsNoClear：初始化完成位是持久状态，读取后不清除，
     * 其他等待者（servo 任务）也能看到。 */
    if (osEventFlagsWait(health_events, RTOS_INIT_DONE, osFlagsNoClear, osWaitForever) != RTOS_INIT_DONE) {
        Error_Handler();
    }
    next = osKernelGetTickCount();
    for (;;) {
        rtos_event(RTOS_BEAT_SENSOR);   /* 喂心跳位，log_health 每 5s 检查 */
        app_hcsr04_process();           /* 触发一次测距并把结果放入单槽邮箱 */
        rtos_wait_period(&next, RTOS_SENSOR_MS);
    }
}

/**
 * @brief 舵机任务：20ms 周期消费命令队列，独占 I2C1/I2C2 执行舵机与步态动作。
 * @param argument 未使用。
 * @note 命令分两类："gait ..." 交给 app_motion（多帧步态插补），
 *       其余（单路角度/off/status）交给 app_servo；执行结果连同原事务号
 *       写入结果队列，由 network 任务回传给 UART4/MQTT 请求方。
 *       I2C 总线只有本任务访问，天然免锁。
 */
static void rtos_servo_task(void *argument)
{
    command_message_t command;
    result_message_t result;
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    /* 初始化期间也会访问 I2C，等待结束后再接管总线。 */
    if (osEventFlagsWait(health_events, RTOS_INIT_DONE, osFlagsNoClear, osWaitForever) != RTOS_INIT_DONE) {
        Error_Handler();
    }
    for (;;) {
        rtos_event(RTOS_BEAT_SERVO);   /* 喂心跳位：即使无命令也每 20ms 报活一次 */
        if (osMessageQueueGet(command_queue, &command, NULL, 0U) == osOK) {
            memset(&result, 0, sizeof(result));
            result.request_id = command.request_id;   /* 事务号原样带回，供回包配对 */
            if (strncmp(command.text, "gait ", 5U) == 0) {
                /* 步态命令：app_motion 自行解析并维护多周期动作状态机 */
                app_motion_command(command.text, result.text, sizeof(result.text), board_io_now());
            } else {
                /* 收到新的单路命令意味着放弃当前步态（人工干预优先）；
                 * "status" 只读查询，不打断正在进行的步态。 */
                if (strcmp(command.text, "status") != 0) { app_motion_cancel(); }
                app_servo_command(command.text, result.text, sizeof(result.text));
            }
            /* 结果入队最多等 20ms（一个任务周期）：给 network 任务一点消费窗口，
             * 仍满则置溢出粘滞位并丢弃，绝不无限阻塞舵机节拍。 */
            if (osMessageQueuePut(result_queue, &result, 0U, rtos_ticks(RTOS_SERVO_WAIT_MS)) != osOK) {
                rtos_event(RTOS_RESULT_OVERFLOW);
            }
        }
        /* 每周期推进步态状态机（无步态时为空操作），实现多帧动作插补 */
        app_motion_process(board_io_now());
        rtos_wait_period(&next, RTOS_SERVO_WAIT_MS);
    }
}

/** @brief 服务任务独占调试串口发送，其他任务只能提交日志队列。 */
/* 日志/健康任务（最低优先级）：
 * 1) 以 20ms 超时轮询日志队列，取到就用 board_io_log_write 直写 USART1
 *    （绕开队列，因为本任务就是队列的唯一消费者，避免递归）；
 * 2) 每 5s 做一次健康报告：一次性读取并清除三个心跳位和两个溢出粘滞位，
 *    打印存活位图（alive=x/7）、四个任务的剩余栈、三个队列的积压深度；
 *    心跳缺位或任一任务栈余量 <256 字节时追加 WARNING 行。 */
static void rtos_service_task(void *argument)
{
    log_message_t message;
    uint32_t last_health = osKernelGetTickCount();
    uint32_t flags;
    uint32_t net_stack, sensor_stack, servo_stack, log_stack;
    char text[RTOS_LOG_BYTES];
    (void)argument;
    board_io_log_write("[RTOS] running: network / servo / distance / log_health\r\n");
    for (;;) {
        /* 20ms 阻塞等待兼顾两件事：有日志时低延迟输出，无日志时让出 CPU，
         * 同时保证健康检查的 5s 计时精度不受轮询影响。 */
        if (osMessageQueueGet(log_queue, &message, NULL, rtos_ticks(20U)) == osOK) {
            board_io_log_write(message.text);
        }
        if ((uint32_t)(osKernelGetTickCount() - last_health) >= rtos_ticks(RTOS_HEALTH_MS)) {
            last_health = osKernelGetTickCount();
            /* Clear-on-read：返回旧值并清零，下个 5s 窗口内任务须重新置位；
             * 某位为 0 即证明对应任务在窗口期内一次都没跑过（停摆/饿死）。 */
            flags = osEventFlagsClear(health_events, RTOS_BEAT_ALL | RTOS_LOG_OVERFLOW | RTOS_RESULT_OVERFLOW);
            if ((flags & osFlagsError) != 0U) { Error_Handler(); }
            /* osThreadGetStackSpace 返回历史最小剩余栈（水位线），非当前用量 */
            net_stack = osThreadGetStackSpace(network_handle);
            sensor_stack = osThreadGetStackSpace(sensor_handle);
            servo_stack = osThreadGetStackSpace(servo_handle);
            log_stack = osThreadGetStackSpace(service_handle);
            (void)snprintf(text, sizeof(text),
                "[RTOS] alive=%lu/7 stack_free(B) net=%lu sensor=%lu servo=%lu log=%lu queues=%lu,%lu,%lu drop=%lu\r\n",
                (unsigned long)(flags & RTOS_BEAT_ALL), (unsigned long)net_stack,
                (unsigned long)sensor_stack, (unsigned long)servo_stack, (unsigned long)log_stack,
                (unsigned long)osMessageQueueGetCount(command_queue),
                (unsigned long)osMessageQueueGetCount(result_queue),
                (unsigned long)osMessageQueueGetCount(log_queue),
                (unsigned long)(flags & (RTOS_LOG_OVERFLOW | RTOS_RESULT_OVERFLOW)));
            board_io_log_write(text);
            /* 告警条件：任一心跳位缺失，或任一任务栈水位低于 256 字节安全线 */
            if ((flags & RTOS_BEAT_ALL) != RTOS_BEAT_ALL || net_stack < RTOS_STACK_WARNING_BYTES ||
                sensor_stack < RTOS_STACK_WARNING_BYTES || servo_stack < RTOS_STACK_WARNING_BYTES ||
                log_stack < RTOS_STACK_WARNING_BYTES) {
                board_io_log_write("[RTOS] WARNING: task heartbeat missing or low stack\r\n");
            }
        }
    }
}

/**
 * @brief 创建全部内核对象（队列 + 事件标志）。由 MX_FREERTOS_Init 在
 *        osKernelInitialize 之后、osKernelStart 之前调用一次。
 * @note distance_queue 深度特意设为 1：它承担"最新值邮箱"角色，
 *       消费者永远只需要最近一次测距，旧样本无保留价值。
 *       任一对象创建失败（FreeRTOS 堆耗尽）即 Error_Handler 停机，
 *       因为缺了队列的系统无法保证任务间数据隔离。
 */
void app_rtos_init(void)
{
    /* rtos_ticks 的换算假设 1ms=1tick；若 CubeMX 配置改动导致 tick 频率
     * 不再是 1000Hz，所有周期定义都会失真，因此在源头断言拦截。 */
    if (osKernelGetTickFreq() != 1000U) { Error_Handler(); }
    log_queue = osMessageQueueNew(RTOS_QUEUE_DEPTH, sizeof(log_message_t), NULL);
    command_queue = osMessageQueueNew(RTOS_QUEUE_DEPTH, sizeof(command_message_t), NULL);
    result_queue = osMessageQueueNew(RTOS_QUEUE_DEPTH, sizeof(result_message_t), NULL);
    distance_queue = osMessageQueueNew(1U, sizeof(app_distance_message_t), NULL);
    health_events = osEventFlagsNew(NULL);
    if (log_queue == NULL || command_queue == NULL || result_queue == NULL ||
        distance_queue == NULL || health_events == NULL) { Error_Handler(); }
    /* 此处只创建内核对象，不访问外设。启动前的 FreeRTOS 临界区可能保留
     * BASEPRI 屏蔽，导致 TIM4 的 HAL tick 不再推进，外设超时无法退出。 */
}

/**
 * @brief 注册 defaultTask 句柄并创建其余三个任务。
 * @param network_thread CubeMX 创建的 defaultTask 句柄（即 network 任务），
 *        在此登记仅为让 log_health 能查询它的栈水位。
 * @note 由 MX_FREERTOS_Init 在 osKernelStart 之前调用；任务创建后立即
 *       阻塞在 RTOS_INIT_DONE 事件上，等 network 任务完成外设初始化。
 */
void app_rtos_create_threads(osThreadId_t network_thread)
{
    network_handle = network_thread;
    sensor_handle = osThreadNew(rtos_sensor_task, NULL, &sensor_attr);
    servo_handle = osThreadNew(rtos_servo_task, NULL, &servo_attr);
    service_handle = osThreadNew(rtos_service_task, NULL, &service_attr);
    if (network_handle == NULL || sensor_handle == NULL || servo_handle == NULL || service_handle == NULL) {
        Error_Handler();
    }
}

/**
 * @brief network 任务主体（在 CubeMX defaultTask 上下文中运行，永不返回）。
 * @note 启动阶段：依次初始化 MQTT(ESP8266/USART3)、舵机(PCA9685/I2C)、
 *       HC-SR04 与 UART4 协议层，然后置 RTOS_INIT_DONE 放行 sensor/servo
 *       任务。初始化必须放在任务里而非 main：I2C 探测等 HAL 阻塞操作依赖
 *       TIM4 提供的 HAL tick，而调度器启动前临界区可能屏蔽该中断。
 *       运行阶段：1ms 周期轮询 MQTT 状态机与 UART4 协议解析，并喂心跳位。
 */
void app_rtos_network_run(void)
{
    uint32_t next;
    /* 调度器启动后才执行依赖 HAL tick 的操作；其他业务任务等待初始化事件。
     * 日志任务已可运行，即使模块未接，I2C 超时也能正常结束并继续启动。 */
    board_io_log("[RTOS] init: network\r\n");
    app_mqtt_init();
    board_io_log("[RTOS] init: servo\r\n");
    app_servo_init();       /* 尝试关闭两块 PCA9685 的残留 PWM，失败不阻塞 */
    board_io_log("[RTOS] init: distance\r\n");
    app_hcsr04_init();
    app_esp_link_init();
    rtos_event(RTOS_INIT_DONE);   /* 放行 sensor/servo 任务，初始化窗口结束 */
    board_io_log("[RTOS] init complete\r\n");
    next = osKernelGetTickCount();
    for (;;) {
        rtos_event(RTOS_BEAT_NET);   /* 喂心跳位 bit0 */
        app_mqtt_process();          /* ESP8266 AT/MQTT 状态机推进 */
        /* 即使 ESP8266 未就绪，UART4 也能独立接收 ESP32 命令。 */
        app_esp_link_process();      /* UART4 协议解析 + 回包发送 + 结果回传 */
        rtos_wait_period(&next, RTOS_NET_MS);
    }
}

/** @brief 内核分配失败或越界时停机，禁止在这些钩子中阻塞发送日志。 */
/* 说明：这两个钩子运行在内核异常上下文，此时调度状态已不可信，
 * 任何阻塞调用（含日志入队）都可能死锁，只能直接 Error_Handler 停机。
 * vApplicationStackOverflowHook 在 configCHECK_FOR_STACK_OVERFLOW 使能时
 * 由内核在任务切换检测到栈越界时回调。 */
void vApplicationMallocFailedHook(void) { Error_Handler(); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    Error_Handler();
}
