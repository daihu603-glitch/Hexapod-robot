/**
 * @file esp_at.c
 * @brief ESP8266 AT 传输层：识别文本应答、+IPD 数据和连接事件，以状态机串行发送。
 *
 * 在系统中的角色：位于 board_io（串口环形缓冲）之上、app_mqtt 之下的传输层。
 * 它把"发一条 AT 命令并等应答""向某条 TCP 连接发数据""关闭连接"抽象成
 * 串行事务，并把接收字节流拆分为三类：AT 文本应答、+IPD 携带的 TCP 载荷、
 * 连接建立/断开事件，分别通过回调或结果码交给上层。ESP8266 AT 固件同一
 * 时刻只能处理一个事务，本层用状态机保证严格串行。
 *
 * AT 状态机（at_state_t）：
 *   IDLE    —— 空闲，无在途事务；
 *   REPLY   —— 已发命令，等待 OK/ERROR 等文本应答；
 *   PROMPT  —— 已发 CIPSEND，等待模块回 '>' 提示符；
 *   PAYLOAD —— 收到 '>'，下一步把原始载荷写进串口；
 *   SEND    —— 载荷已发出，等待 SEND OK/SEND FAIL。
 */
#include "esp_at.h"
#include "board_io.h"
#include <stdio.h>
#include <string.h>

/* 按普通传输、多连接和不附加远端信息的格式处理 AT 数据；具体固件兼容由应用初始化负责。 */
typedef enum { AT_IDLE, AT_REPLY, AT_PROMPT, AT_PAYLOAD, AT_SEND } at_state_t;
static at_state_t state;
static esp_data_fn on_data;      /* TCP 载荷回调（逐字节交给 mqtt_endpoint） */
static esp_link_fn on_link;      /* 连接建立/断开回调（交给 app_mqtt） */
static char line[240];           /* AT 文本行累积缓冲 */
static size_t line_length;
static uint8_t payload[512];     /* 待 CIPSEND 的原始载荷暂存 */
static size_t payload_length;
static uint32_t started;         /* 当前事务起始时刻，用于命令超时 */
static uint32_t timeout;         /* 当前事务允许的毫秒超时 */
static uint32_t ipd_started;     /* 当前 +IPD 载荷接收起始时刻 */
static uint32_t ipd_remaining;   /* 当前 +IPD 还需按原始数据处理的字节数 */
static uint8_t ipd_link;         /* 当前 +IPD 所属连接编号 */
static int completion;           /* 上次事务结果：0 无/等待，1 成功，-1 错误，-2 超时/故障 */
static int fault;                /* 传输层故障标志，需上层复位网络 */
static int commands_started;     /* 是否已发过命令，用于区分正常启动 ready 与意外重启 ready */
static uint8_t send_link;        /* 当前 CIPSEND 目标连接，用于断开事件校验 */
/* 命令缓冲区由主循环独占，通过 esp_at_busy 保证命令串行执行。 */
static char command_buffer[240];

/**
 * @brief 注册数据与连接回调，清空 AT 状态和上次事务结果。
 */
void esp_at_init(esp_data_fn data_callback, esp_link_fn link_callback)
{
    on_data = data_callback;
    on_link = link_callback;
    state = AT_IDLE;
    line_length = 0U;
    ipd_remaining = 0U;
    completion = 0;
    fault = 0;
    commands_started = 0;
}

/**
 * @brief 结束当前 AT 事务并保存结果，等待上层调用 esp_at_result 消费。
 */
static void finish(int result)
{
    state = AT_IDLE;
    completion = result;
}

/**
 * @brief 命令执行中、结果尚未读取或串口仍在发送时返回非零。
 */
int esp_at_busy(void) { return state != AT_IDLE || completion != 0 || board_io_wifi_tx_busy(); }
/**
 * @brief 查询传输故障；出现故障后由上层复位网络并重新初始化。
 */
int esp_at_fault(void) { return fault; }
/**
 * @brief 取出并清除事务结果：0 等待或空闲，1 成功，-1 命令错误，-2 超时或故障。
 */
int esp_at_result(void) { int result = completion; completion = 0; return result; }

/**
 * @brief 追加回车换行并启动一条 AT 命令；timeout_ms 指定整个事务的超时时间。
 */
int esp_at_command(const char *command, uint32_t timeout_ms)
{
    int length;
    if (!command || esp_at_busy() || fault) { return 0; }
    length = snprintf(command_buffer, sizeof(command_buffer), "%s\r\n", command);
    if (length <= 0 || (size_t)length >= sizeof(command_buffer)) { return 0; }
    if (!board_io_wifi_send((const uint8_t *)command_buffer, (size_t)length)) { return 0; }
    started = board_io_now();
    timeout = timeout_ms;
    state = AT_REPLY;
    commands_started = 1;
    return 1;
}

/**
 * @brief 复制网络载荷并发送 CIPSEND；等待提示符后由处理函数发送实际数据。
 *
 * AT 发送数据是三段式流程：
 * 1) 本函数发出 "AT+CIPSEND=<link>,<length>\r\n" 并把状态置为 AT_PROMPT；
 * 2) 模块回 '>' 提示符，receive_byte 转入 AT_PAYLOAD；
 * 3) esp_at_process 发现进入 PAYLOAD 且串口空闲后，把暂存的 payload 作为
 *    原始字节（不再是 AT 命令）写入串口，转入 AT_SEND 等待 "SEND OK"。
 * link 上限 5（ESP8266 AT+CIPMUX=1 支持连接编号 0..4）；载荷先复制到
 * 静态 payload，因为提示符到来之前调用方的缓冲可能已经失效。
 */
int esp_at_send(uint8_t link, const uint8_t *data, size_t length)
{
    char command[40];
    if (link >= 5U || !data || !length || length > sizeof(payload) || esp_at_busy() || fault) { return 0; }
    (void)snprintf(command, sizeof(command), "AT+CIPSEND=%u,%u", (unsigned)link, (unsigned)length);
    if (!esp_at_command(command, 5000U)) { return 0; }
    memcpy(payload, data, length);
    payload_length = length;
    send_link = link;
    state = AT_PROMPT;
    return 1;
}

/**
 * @brief 为编号 0～4 的 TCP 连接发起关闭命令。
 */
int esp_at_close(uint8_t link)
{
    char command[32];
    if (link >= 5U) { return 0; }
    (void)snprintf(command, sizeof(command), "AT+CIPCLOSE=%u", (unsigned)link);
    return esp_at_command(command, 3000U);
}

/**
 * @brief 解析完整文本行，区分连接通知、启动提示、命令完成和错误应答。
 *
 * 行类型与处理：
 * - "<n>,CONNECT" / "<n>,CLOSED"（长度恰为 9/8）：多连接模式下模块主动上报
 *   的连接事件，提取连接号后回调 on_link，供 app_mqtt 维护会话与代次；
 * - "ready"：模块启动完成提示。首条命令前收到属正常启动流程；命令执行期间
 *   收到说明模块意外重启（掉电/看门狗），置 fault 强制上层复位重来；
 * - "OK"：普通命令成功，仅在 AT_REPLY 态消费，防止 +IPD 数据里伪造的
 *   "OK" 行干扰（载荷段根本不会走到这里）；
 * - "SEND OK"：CIPSEND 数据发送成功，仅在 AT_SEND 态消费；
 * - "ERROR"/"FAIL"/"SEND FAIL"/"busy..."：命令级失败，以 -1 结束事务，
 *   上层可据此走旧语法回退等策略；
 * - 版本信息行（AT+GMR / AT+CIFSR 的应答）：仅记日志便于排查，不影响状态。
 */
static void parse_line(void)
{
    line[line_length] = '\0';
    if (line_length == 9U && line[0] >= '0' && line[0] <= '4' && strcmp(line + 1, ",CONNECT") == 0) {
        if (on_link) { on_link((uint8_t)(line[0] - '0'), 1); }
    } else if (line_length == 8U && line[0] >= '0' && line[0] <= '4' && strcmp(line + 1, ",CLOSED") == 0) {
        /* 连接断开时模块可能仍等待载荷；将状态标记为故障，交由上层复位，
           避免把旧数据或新 AT 命令误作为未完成 CIPSEND 的载荷。 */
        if ((state == AT_PROMPT || state == AT_PAYLOAD || state == AT_SEND) &&
            send_link == (uint8_t)(line[0] - '0')) { fault = 1; }
        if (on_link) { on_link((uint8_t)(line[0] - '0'), 0); }
    } else if (strcmp(line, "ready") == 0) {
        /* 首次命令前的 ready 属于正常启动；命令开始后的 ready 表示模块意外重启。 */
        if (commands_started) { fault = 1; }
    } else if (strcmp(line, "OK") == 0) {
        if (state == AT_REPLY) { finish(1); }
    } else if (strcmp(line, "SEND OK") == 0) {
        if (state == AT_SEND) { finish(1); }
    } else if (strcmp(line, "ERROR") == 0 || strcmp(line, "FAIL") == 0 || strcmp(line, "SEND FAIL") == 0 || strncmp(line, "busy", 4U) == 0) {
        if (state != AT_IDLE) { finish(-1); }
    } else if (strncmp(line, "AT version", 10U) == 0 || strncmp(line, "SDK version", 11U) == 0 || strncmp(line, "version:", 8U) == 0 || strncmp(line, "+CIFSR:", 7U) == 0) {
        board_io_log(line);
        board_io_log("\r\n");
    }
    line_length = 0U;
}

/**
 * @brief 解析 +IPD,<连接号>,<长度>:，记录后续应按原始数据处理的字节数。
 *
 * 该头部以 ':' 结束（由 receive_byte 在累积时检测触发解析）。多连接且
 * CIPDINFO=0 的固件格式固定为 "+IPD,<0-4>,<len>:"。解析要点：
 * - 连接号必须是单字符 '0'..'4'，否则视为流损坏置 fault；
 * - 长度按十进制逐位累加，超过 65535（TCP 单段上限）判为非法；
 * - 成功后清空行缓冲并把 ipd_remaining 设为长度，之后 ipd_remaining 个
 *   字节全部作为 TCP 原始载荷直通 on_data，即使其中出现 '\n'、'>' 或
 *   "OK" 也不会被误当作 AT 应答——这是文本协议中携带二进制数据的关键。
 * 返回 1 表示成功进入载荷接收；返回 0 表示不是 +IPD 头（按普通行忽略）
 * 或格式非法（已置 fault）。
 */
static int parse_ipd_header(void)
{
    size_t pos;
    uint32_t length = 0U;
    if (line_length < 9U || memcmp(line, "+IPD,", 5U) != 0) { return 0; }
    if (line[5] < '0' || line[5] > '4' || line[6] != ',') { fault = 1; return 0; }
    for (pos = 7U; pos + 1U < line_length; ++pos) {
        if (line[pos] < '0' || line[pos] > '9') { fault = 1; return 0; }
        length = length * 10U + (uint32_t)(line[pos] - '0');
        if (length > 65535U) { fault = 1; return 0; }
    }
    ipd_link = (uint8_t)(line[5] - '0');
    ipd_remaining = length;
    ipd_started = board_io_now();
    line_length = 0U;
    return 1;
}

/**
 * @brief 逐字节分离 AT 文本与 TCP 原始数据；原始数据中的换行和提示符不作为 AT 应答。
 *
 * 处理优先级（自上而下）：
 * 1) ipd_remaining > 0：处于 +IPD 载荷段，字节无条件直通 on_data 回调；
 * 2) '>' 且处于 AT_PROMPT：这是 CIPSEND 的发送提示符，转 AT_PAYLOAD；
 *    其他状态下的 '>' 只是普通文本字符，走行累积；
 * 3) '\r' 丢弃（模块应答统一以 \r\n 结尾，'\n' 作为行结束）；
 * 4) 行首空格丢弃，兼容部分固件在应答前输出的空格；
 * 5) 行超长（>=239 字节仍无换行）说明字节流已错位，置 fault 交上层复位；
 * 6) 每累积一个 ':' 就检查行首是否为 "+IPD,"，是则尝试解析载荷头——
 *    +IPD 头以 ':' 结束且后面紧跟二进制数据，不能等 '\n' 再解析。
 */
static void receive_byte(uint8_t byte)
{
    if (ipd_remaining) {
        --ipd_remaining;
        if (on_data) { on_data(ipd_link, byte); }
        return;
    }
    if (byte == '>' && state == AT_PROMPT) { state = AT_PAYLOAD; line_length = 0U; return; }
    if (byte == '\r') { return; }
    if (byte == '\n') { parse_line(); return; }
    if (line_length == 0U && byte == ' ') { return; }
    if (line_length >= sizeof(line) - 1U) { fault = 1; return; }
    line[line_length++] = (char)byte;
    if (byte == ':' && line_length >= 5U && memcmp(line, "+IPD,", 5U) == 0) { (void)parse_ipd_header(); }
}

/**
 * @brief 主循环处理接收队列和发送阶段，每次限制处理预算，并检测数据与命令超时。
 *
 * 每轮最多消费 4096 字节（与接收环形缓冲同容量），保证单次调用有界，
 * 不会因数据突发饿死其他任务。两级超时保护：
 * - +IPD 载荷段超过 3s 未收齐：说明字节流错位或模块异常，置 fault；
 * - AT 事务整体超过发起时指定的 timeout：置 fault。
 * fault 一旦置位，在途事务以 -2（超时/故障）结束，app_mqtt 检测到后
 * 复位 ESP8266 重新初始化——传输层自身不做局部恢复，避免状态不一致。
 */
void esp_at_process(void)
{
    uint8_t byte;
    unsigned budget = 4096U;
    if (board_io_wifi_fault()) { board_io_log("[AT] UART receive error/overflow\r\n"); fault = 1; }
    while (budget-- && !fault && board_io_wifi_read(&byte)) { receive_byte(byte); }
    if (!fault && state == AT_PAYLOAD && !board_io_wifi_tx_busy()) {
        if (board_io_wifi_send(payload, payload_length)) { state = AT_SEND; }
        else { fault = 1; }
    }
    if (ipd_remaining && (uint32_t)(board_io_now() - ipd_started) > 3000U) { fault = 1; }
    if (state != AT_IDLE && (uint32_t)(board_io_now() - started) > timeout) {
        board_io_log("[AT] command response timeout\r\n");
        fault = 1;
    }
    if (fault && state != AT_IDLE) { finish(-2); }
}
