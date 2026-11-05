/** @file app_esp_link.c
 * @brief UART4 双向低速命令，115200/8N1；@事务号 命令，对应 @事务号 OK/ERR 结果。
 * LF 或 CR 结束一行；CRLF 也支持。出错后丢弃到行尾，禁止执行残缺命令。
 * 仅网络任务调用；日志经 USART1 输出，舵机 I2C 仍由原舵机任务独占。 */
/* 【中文详解】本模块是 STM32F103 与 ESP32-S3 之间 UART4 文本协议的解析器。
 * 协议格式：
 *   请求行  "@<事务号> <命令>"（事务号为 1~6 位十进制，0 非法；命令白名单：
 *           ping / distance / status / gait ... / servo ...）
 *   响应行  "@<事务号> OK|ERR|WAIT <结果文本>\n"
 * 事务流程：先回 "WAIT queued" 表示已入队，舵机任务执行完成后经
 * app_rtos_servo_result -> app_esp_link_result 再回最终 OK/ERR 结果，
 * 同一事务号因此有两条回包（这也是发送队列要为每个事务预留两格的原因）。
 * 可靠性设计：任何解析/校验/传输错误都只作废"到行尾为止"的字节流，
 * 绝不把残缺内容拼接成可执行命令（失效安全）。 */
#include "app_esp_link.h"
#include "app_rtos.h"
#include "app_mqtt.h"
#include "board_io.h"
#include "app_hcsr04.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* 单行接收缓冲上限：64 字节足够容纳最长命令 "gait ..."，也防止 ESP32 侧
 * 异常刷屏时耗尽内存；超长行会被整行丢弃而不是截断执行。 */
#define ESP_LINK_LINE_BYTES 64U
/* 每次 process() 调用最多读取的字节数：1ms 轮询周期下 115200bps 理论到达
 * 约 11 字节/ms，128 的上限保证即使任务被延迟调度也能一次排空 RX 环形缓冲，
 * 同时限制单次调用耗时，避免饿死同任务里的 MQTT 处理。 */
#define ESP_LINK_POLL_BYTES 128U
/* 行内字节间隔超时：两字节之间静默超过 250ms 视为半行作废。取 250ms 是
 * 因为 115200bps 下正常字节间隔为微秒级，250ms 足以覆盖 ESP32 侧任务调度
 * 抖动，又能及时清掉断线/噪声产生的残行。 */
#define ESP_LINK_GAP_MS 250U

static char esp_line[ESP_LINK_LINE_BYTES];   /* 当前行拼装缓冲，仅在行结束符处消费 */
static size_t esp_length;                     /* esp_line 中已接收的有效字节数 */
static uint32_t esp_last_byte_ms;             /* 最后一个字节到达时刻，用于行超时判断 */
static uint8_t esp_discard;                   /* 非零表示本行已作废，丢弃直到下一个 CR/LF */
/* 仅通信任务访问，ISR 不接触协议队列。拥塞时拒绝新事务，不重复执行动作。 */
/* 回包发送环形队列：8 深 × 160 字节。8 深对应最多 4 个并发事务（每个事务
 * WAIT+最终结果两条回包）；board_io_esp_send 忙时会失败，报文须留在队列中
 * 由下一轮 process() 重试，因此必须本地缓存而不能直接发送。 */
static char replies[8U][160U];
static unsigned reply_head, reply_tail;       /* head=写入位置，tail=发送位置，空一格区分满/空 */

/**
 * @brief 组装一条 "@<事务号> <状态> <文本>\n" 回包并压入发送环形队列。
 * @param id     事务号；0 表示无编号请求，按协议不回包（单向命令）。
 * @param status "OK"/"ERR"/"WAIT" 三态字符串。
 * @param text   结果文本，与状态一起被 snprintf 截断到 160 字节内。
 * @note 队列满时丢弃本条回包并记日志：宁可让 ESP32 侧超时重试，
 *       也不能阻塞通信任务或覆盖尚未发出的旧回包。
 *       仅网络任务调用，无需加锁。
 */
static void esp_reply(uint32_t id, const char *status, const char *text)
{
    unsigned next = (reply_head + 1U) % 8U;
    if (id == 0U) { return; }   /* 无事务号的请求不需要（也无法）匹配回包 */
    if (next == reply_tail) {   /* 环形队列满（保留一格判满）：丢弃并留痕 */
        board_io_log("[ESP32] reply queue full\r\n");
        return;
    }
    (void)snprintf(replies[reply_head], sizeof(replies[0]), "@%lu %s %s\n",
                   (unsigned long)id, status, text);
    reply_head = next;
}

/**
 * @brief 舵机/步态命令执行完毕后的最终结果回传入口（由 app_rtos_servo_result 调用）。
 * @param request_id 原请求的事务号。
 * @param result     执行层生成的结果文本；以 "error" 开头映射为 ERR，否则为 OK。
 * @note 仅网络任务调用。这里是 "WAIT queued" 之后的第二条回包，
 *       ESP32 侧靠事务号把它与最初请求配对。
 */
void app_esp_link_result(uint32_t request_id, const char *result)
{
    if (result == NULL) { return; }
    /* 执行层约定：失败文本统一以 "error" 前缀开头，据此选择 ERR/OK 状态字。 */
    esp_reply(request_id, strncmp(result, "error", 5U) == 0 ? "ERR" : "OK", result);
}

/** @brief 严格读取最多三位十进制数；不接受负号、尾随空格或多余字段。 */
/* 三位上限对应舵机角度最大 135 与编号最大 17；严格解析（成功时必须恰好停在
 * 分隔符/串尾）避免 "90abc" 这类残缺输入被当作合法角度执行。
 * @param cursor 输入/输出：当前解析位置，成功时前移到数字之后。
 * @param value  输出：解析出的无符号数值。
 * @return 1 成功；0 表示无数字或超过三位。 */
static int esp_number(const char **cursor, unsigned *value)
{
    const char *p = *cursor;
    unsigned n = 0U, digits = 0U;
    while (*p >= '0' && *p <= '9') {
        if (++digits > 3U) { return 0; }   /* 超过三位即非法，防止越界数值 */
        n = n * 10U + (unsigned)(*p++ - '0');
    }
    if (digits == 0U) { return 0; }        /* 一个数字都没有也是非法 */
    *cursor = p;
    *value = n;
    return 1;
}

/** @brief 入队前验证编号与角度；实际执行层仍保留原有二次校验。 */
/* 白名单校验 servo 载荷：<id 0..17> <angle 45..135|off>，或整板 "off"/"status"。
 * 提前在通信任务拒绝非法命令有两个目的：
 * 1) 非法输入不占用有限的 8 深命令队列；
 * 2) 可以立即回 ERR，而不必先 WAIT 再等执行层报错。
 * @return 1 合法；0 非法。 */
static int esp_servo_valid(const char *payload)
{
    const char *p = payload;
    unsigned id, angle;
    /* 全局命令：关全部输出 / 查询配置状态，不带参数 */
    if (strcmp(p, "off") == 0 || strcmp(p, "status") == 0) { return 1; }
    /* "<id> " 形式：编号必须 0..17 且其后紧跟单个空格 */
    if (!esp_number(&p, &id) || id >= 18U || *p != ' ') { return 0; }
    ++p;
    if (strcmp(p, "off") == 0) { return 1; }   /* "<id> off"：单独关断某路 */
    /* "<id> <angle>"：角度限定 45..135（对应 1000..2000us 脉宽的机械安全区间），
     * 且数字后必须是串尾，禁止尾随垃圾字符。 */
    return esp_number(&p, &angle) && *p == '\0' && angle >= 45U && angle <= 135U;
}

/** @brief 只接受白名单命令；ping 不触发舵机，status 只查询配置标志。 */
/* 解析并分发一个完整的命令行。步骤：
 * 1) 若以 '@' 开头，解析事务号（1~6 位十进制，非零），其后必须紧跟空格；
 *    事务号用于把 WAIT/OK/ERR 回包与 ESP32 侧的未完成请求配对。
 * 2) 检查回包队列至少还有 2 个空位——一个事务最多产生 "WAIT queued" 和
 *    最终结果两条回包，预留不足则拒绝接收，杜绝"执行了动作却无法回包"。
 * 3) 按白名单分发：ping（本地应答）、distance（读 HC-SR04 队列）、
 *    status/gait/servo（复制载荷后投递到舵机命令队列，由舵机任务执行）。
 * 无 '@' 前缀的裸命令兼容旧的单向调试格式（id=0，不回包）。 */
static void esp_dispatch(void)
{
    const char *payload;
    const char *command;
    uint32_t id = 0U;
    unsigned digits = 0U;
    char distance[40];
    char peripheral_reply[128];
    esp_line[esp_length] = '\0';   /* 行缓冲以 NUL 收尾，后续可直接用 str* 函数 */
    command = esp_line;
    if (*command == '@') {
        ++command;
        /* 事务号最多 6 位（0~999999），超过视为非法行直接丢弃 */
        while (*command >= '0' && *command <= '9') {
            if (++digits > 6U) { return; }
            id = id * 10U + (uint32_t)(*command++ - '0');
        }
        /* 事务号必须存在、非零，且与命令之间恰好一个空格；0 保留给"无编号"语义 */
        if (digits == 0U || id == 0U || *command++ != ' ') { return; }
        /* 保留至少两格用于 queued 与最终结果；不接收无回包能力的动作。 */
        /* 计算式说明：环形队列空一格判满，(tail + 8 - head - 1) % 8 即当前
         * 可用空位数，小于 2 就拒绝整个事务。 */
        if ((reply_tail + 8U - reply_head - 1U) % 8U < 2U) { return; }
    }
    if (strcmp(command, "ping") == 0) {
        /* 链路心跳：完全在通信任务本地应答，不经过舵机队列，
         * 因此即使舵机任务卡住也能判断 UART4 链路是否存活。 */
        board_io_log("[ESP32] pong (UART4)\r\n");
        esp_reply(id, "OK", "pong");
        return;
    }
    if (strcmp(command, "distance") == 0) {
        /* 取 HC-SR04 最近一次测距结果（500ms 周期采集，数据已在采集任务格式化）；
         * 队列中没有新样本时回 ERR，不阻塞等待。 */
        if (app_hcsr04_format(distance, sizeof(distance))) { esp_reply(id, "OK", distance); }
        else { esp_reply(id, "ERR", "distance unavailable"); }
        return;
    }
    if (app_mqtt_link_command(command, peripheral_reply, sizeof(peripheral_reply))) {
        esp_reply(id, strncmp(peripheral_reply, "error", 5U) == 0 ? "ERR" : "OK",
                  peripheral_reply);
        return;
    }
    if (strcmp(command, "status") == 0) { payload = "status"; }
    /* "gait xxx"：整行作为载荷（步态命令由 app_motion 解析）；
     * "servo xxx"：剥掉 "servo " 前缀后作为舵机载荷，兼容 ESP32 侧的统一格式。 */
    else if (strncmp(command, "gait ", 5U) == 0) { payload = command; }
    else if (strncmp(command, "servo ", 6U) == 0) { payload = command + 6U; }
    else {
        /* 白名单之外的命令一律拒绝并记录，不回显原文，防止日志注入。 */
        esp_reply(id, "ERR", "unknown command");
        board_io_log("[ESP32] rejected: unknown command\r\n"); return;
    }
    /* gait 命令的参数校验在 app_motion 层完成；servo 载荷在此做入队前预校验 */
    if (strncmp(payload, "gait ", 5U) != 0 && !esp_servo_valid(payload)) {
        board_io_log("[ESP32] rejected: servo ID 0..17, angle 45..135 or off\r\n");
        esp_reply(id, "ERR", "invalid servo command");
        return;
    }
    /* 队列会复制 payload，解析缓冲可以立即复用；队列满则拒绝，不隐式重试动作。 */
    /* id==0（无事务号）走普通提交；id!=0 走带编号提交，执行结果会经
     * app_rtos_servo_result -> app_esp_link_result 回传给对应事务。 */
    if (!(id == 0U ? app_rtos_servo_submit(payload) : app_rtos_servo_submit_link(payload, id))) {
        board_io_log("[ESP32] rejected: servo queue full\r\n");
        esp_reply(id, "ERR", "servo queue full");
        return;
    }
    /* 入队成功先回 WAIT：告诉 ESP32 "已受理、尚未执行"，最终结果随后到达。 */
    board_io_log("[ESP32] queued servo command\r\n");
    esp_reply(id, "WAIT", "queued");
}

/**
 * @brief 初始化 UART4 协议解析状态（复位行缓冲与回包队列指针）。
 * @note 由网络任务在 app_rtos_network_run 的启动序列中调用一次；
 *       必须在 board_io/UART4 外设初始化之后。只清软件状态，
 *       不触碰硬件、不移动舵机。
 */
void app_esp_link_init(void)
{
    esp_length = 0U;
    reply_head = reply_tail = 0U;
    esp_discard = 0U;
    esp_last_byte_ms = board_io_now();   /* 以当前时刻为基准，避免上电残留时间戳触发假超时 */
    board_io_log("[ESP32] UART4 duplex ready: TX=PC10 RX=PC11 115200 8N1\r\n");
}

/**
 * @brief UART4 协议周期处理：先发一条待发回包，再收最多 128 字节拼行解析。
 * @note 仅网络任务以 1ms 周期调用（app_rtos_network_run 主循环）。
 *       非阻塞设计：发送忙时回包留在队列下轮重试；接收按行状态机处理，
 *       任何异常（错误/溢出/非法字符/超时）都进入"丢弃到行尾"模式，
 *       保证残缺字节流永远不会拼成可执行命令。
 */
void app_esp_link_process(void)
{
    uint8_t byte;
    unsigned count;
    int received;
    uint32_t now = board_io_now();
    /* 每轮最多发送一条回包：board_io_esp_send 是中断发送，忙时返回 0，
     * 此时不出队，下一轮 1ms 后重试；成功才推进 tail。 */
    if (reply_tail != reply_head && board_io_esp_send(replies[reply_tail])) {
        reply_tail = (reply_tail + 1U) % 8U;
    }
    /* 超时只作废半行，不把后续尾巴当作新命令；必须先收到行结束符。 */
    /* 无符号减法 (now - last) 天然兼容毫秒计数器约 49.7 天的回绕。 */
    if (esp_length != 0U && (uint32_t)(now - esp_last_byte_ms) >= ESP_LINK_GAP_MS) {
        esp_length = 0U;
        esp_discard = 1U;   /* 置丢弃态：本行残余字节全部忽略，直到 CR/LF 才复位 */
        board_io_log("[ESP32] rejected: incomplete line timeout\r\n");
    }
    for (count = 0U; count < ESP_LINK_POLL_BYTES; ++count) {
        received = board_io_esp_read(&byte);
        if (received == 0) { break; }   /* RX 环形缓冲已排空，本轮结束 */
        if (received < 0) {
            /* 底层报告 UART 错误/溢出（已尝试重启接收）：当前行不可信，
             * 丢弃到行尾，防止错误边界之后的字节被误拼进命令。 */
            esp_length = 0U;
            esp_discard = 1U;
            board_io_log("[ESP32] UART4 error/overflow: discard until newline\r\n");
            break;
        }
        esp_last_byte_ms = now;   /* 刷新行内活跃时间戳，用于 250ms 半行超时 */
        if (byte == '\r' || byte == '\n') {
            /* 行结束符：CRLF 会产生两次进入，第二次 length==0 不会重复派发。
             * 只有未作废且非空的行才交给 esp_dispatch。 */
            if (!esp_discard && esp_length != 0U) { esp_dispatch(); }
            esp_length = 0U;
            esp_discard = 0U;   /* 无论上一行状态如何，新行从头开始 */
        } else if (!esp_discard) {
            /* 可打印 ASCII（32~126）之外或超出 64 字节缓冲的行整行作废：
             * 丢弃模式下继续吞掉字节但不入缓冲，直到行结束符。 */
            if (byte < 32U || byte > 126U || esp_length + 1U >= sizeof(esp_line)) {
                esp_discard = 1U;
                esp_length = 0U;
                board_io_log("[ESP32] rejected: non-ASCII or oversized line\r\n");
            } else { esp_line[esp_length++] = (char)byte; }
        }
    }
}
