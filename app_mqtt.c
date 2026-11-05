/**
 * @file app_mqtt.c
 * @brief 应用调度：建立 ESP8266 热点和本地 MQTT 服务，统一处理手机与串口控制命令。
 *
 * 在系统中的角色：本文件是整机的主控状态机，串起 board_io（硬件）、
 * esp_at（AT 传输）、mqtt_endpoint（本地 MQTT 服务端）、app_hcsr04（超声波）
 * 与 app_rtos（舵机/日志任务）。它把 ESP8266 配成 Wi-Fi 热点（SoftAP），
 * 手机连上后经 MQTT 下发舵机/RGB/蜂鸣器命令，STM32 再把结果和状态回发。
 *
 * 应用状态机（app_state_t）：
 *   APP_FAILED —— 初始化失败，停机等待人工干预；
 *   APP_RESET  —— 已拉低 ESP8266 复位脚，保持 100ms 后释放；
 *   APP_BOOT   —— 释放复位后等待 2s，让模块完成上电启动再握手；
 *   APP_SETUP  —— 逐条下发 AT 命令配置热点、DHCP 与 TCP 服务器；
 *   APP_READY  —— 服务就绪，处理 MQTT 报文、命令分发与状态上报；
 *   APP_RETRY  —— 传输故障，等待 3s 后重新复位模块，回到 APP_RESET。
 */
#include "app_mqtt.h"
#include "app_config.h"
#include "board_io.h"
#include "esp_at.h"
#include "mqtt_endpoint.h"
#include "app_hcsr04.h"
#include "app_rtos.h"
#include <stdio.h>
#include <string.h>

#define QUEUE_COUNT 16U         /* 待发报文环形队列深度 */
#define PACKET_CAPACITY 512U    /* 单个待发报文最大字节数 */
#define SETUP_COMMANDS 15U      /* setup_command 中 case 0..14 的命令总数 */

typedef enum { APP_FAILED, APP_RESET, APP_BOOT, APP_SETUP, APP_READY, APP_RETRY } app_state_t;
typedef struct {
    uint32_t generation;        /* 入队时的连接代次，出队时校验连接是否已换代 */
    uint16_t length;
    uint8_t link;               /* ESP8266 多连接编号 0..4 */
    uint8_t bytes[PACKET_CAPACITY];
} queued_packet_t;

static queued_packet_t queue[QUEUE_COUNT];
static uint8_t queue_head;
static uint8_t queue_tail;
/* 连接代次机制：每条 TCP 连接断开或重建都令 generation[link] 自增。
 * 报文入队时记录当时代次，出队或完成回调时若发现代次已变，说明该报文
 * 属于旧连接，直接丢弃，绝不误发给恰好复用了同一 link 编号的新连接。 */
static uint32_t generation[MQTT_CLIENTS];
static uint8_t tcp_open[MQTT_CLIENTS];       /* 各 link 的 TCP 是否处于打开状态 */
static uint8_t pending_close[MQTT_CLIENTS];  /* 请求关闭但待发报文尚未清空 */
static app_state_t state;
static uint32_t state_started;   /* 进入当前状态的时刻，用于各状态超时计时 */
static uint32_t last_status;     /* 上次周期状态上报时刻，间隔 2s */
static uint8_t setup_index;      /* 当前 AT 配置步骤序号（0..SETUP_COMMANDS） */
static uint8_t setup_waiting;    /* 已发出配置命令、正等待 AT 应答 */
static uint8_t legacy_command;   /* 当前步骤回退到旧版 AT 语法（无 _CUR 后缀） */
static uint8_t operation; /* 0 空闲，1 正在发送，2 正在关闭连接 */
static uint8_t operation_link;
static uint32_t operation_generation;
static uint8_t state_dirty;      /* 置位表示状态变化，需立即而非等到 2s 才上报 */
static char result_text[80];     /* 最近一条命令的执行结果，待发布到 guide/result */
static char console_line[96];    /* 调试串口行缓冲 */
static size_t console_length;
static int console_discard;      /* 当前行超长，丢弃直到下一个换行 */

/**
 * @brief 切换应用状态，并记录进入时间，供复位、启动和重试计时使用。
 */
static void change_state(app_state_t next)
{
    state = next;
    state_started = board_io_now();
}

/**
 * @brief 复制待发报文到环形队列；记录连接代次，避免旧连接报文发给复用同一编号的新连接。
 *
 * 作为 mqtt_endpoint 的发送回调注入。拒绝条件：link 越界、连接未打开、
 * 空数据、超过单包容量、或队列已满（head 追平 tail），均返回 0。队列满时
 * 上层会断开该客户端，属于背压保护，防止慢连接拖垮整机。
 */
static int enqueue(uint8_t link, const uint8_t *data, size_t length)
{
    uint8_t next = (uint8_t)((queue_head + 1U) % QUEUE_COUNT);
    queued_packet_t *packet;
    if (link >= MQTT_CLIENTS || !tcp_open[link] || !data || length > PACKET_CAPACITY || next == queue_tail) { return 0; }
    packet = &queue[queue_head];
    packet->link = link;
    packet->generation = generation[link];
    packet->length = (uint16_t)length;
    memcpy(packet->bytes, data, length);
    queue_head = next;
    return 1;
}

/**
 * @brief 登记关闭请求，由主循环在待发报文处理完成后执行。
 */
static void request_close(uint8_t link)
{
    if (link < MQTT_CLIENTS && tcp_open[link]) { pending_close[link] = 1U; }
}

/**
 * @brief 解析蜂鸣器持续时间：只接受十进制数字，范围为 0～1000 毫秒。
 */
static int parse_duration(const char *text, uint32_t *duration)
{
    uint32_t value = 0U;
    size_t i;
    if (!text[0]) { return 0; }
    for (i = 0U; text[i]; ++i) {
        if (text[i] < '0' || text[i] > '9' || i >= 4U) { return 0; }
        value = value * 10U + (uint32_t)(text[i] - '0');
    }
    if (value > 1000U) { return 0; }
    *duration = value;
    return 1;
}

/**
 * @brief 按主题分发舵机、RGB 灯和蜂鸣器命令，将执行结果写入公共回复缓冲区。
 *
 * 命令分发流程：MQTT PUBLISH（或串口命令）→ mqtt_endpoint 校验后回调本函数
 * → 按主题匹配执行 → 结果文本写入 result_text → app_mqtt_process 末尾统一
 * 发布到 guide/result 主题并打印日志。三个主题：
 *   APP_TOPIC_SERVO：舵机/步态命令，转投舵机任务队列（跨任务异步执行）；
 *   APP_TOPIC_RGB：颜色名直接映射 3 位掩码，立即生效；
 *   APP_TOPIC_BEEP：鸣响毫秒数或 off，立即生效。
 * 未匹配的主题静默忽略，非法载荷回复 error 提示可用取值。
 */
static void handle_command(const char *topic, const char *payload)
{
    static const char *const colors[] = {"off", "red", "green", "yellow", "blue", "magenta", "cyan", "white"};
    unsigned i;
    uint32_t duration;
    if (strcmp(topic, APP_TOPIC_SERVO) == 0) {
        /* I2C 归舵机任务独占；这里仅入队，实际执行结果异步返回 guide/result。 */
        (void)snprintf(result_text, sizeof(result_text), "%s",
                       app_rtos_servo_submit(payload) ? "queued servo command" : "error servo queue full/invalid");
    } else if (strcmp(topic, APP_TOPIC_RGB) == 0) {
        for (i = 0U; i < 8U; ++i) {
            if (strcmp(payload, colors[i]) == 0) {
                board_io_rgb((uint8_t)i);
                (void)snprintf(result_text, sizeof(result_text), "ok rgb %s", colors[i]);
                state_dirty = 1U;
                return;
            }
        }
        strcpy(result_text, "error rgb: off/red/green/blue/yellow/cyan/magenta/white");
    } else if (strcmp(topic, APP_TOPIC_BEEP) == 0) {
        if (strcmp(payload, "off") == 0) { duration = 0U; }
        else if (!parse_duration(payload, &duration)) { strcpy(result_text, "error beep: 0..1000 milliseconds or off"); return; }
        board_io_beep(duration);
        (void)snprintf(result_text, sizeof(result_text), "ok beep %lu ms", (unsigned long)duration);
        state_dirty = 1U;
    }
}

/**
 * @brief 仅在网络就绪后，将指定 TCP 连接收到的字节交给 MQTT 组包器。
 */
static void received_byte(uint8_t link, uint8_t byte)
{
    if (state == APP_READY) { mqtt_endpoint_feed(link, byte, board_io_now()); }
}

/**
 * @brief 响应 TCP 建立或断开事件，更新连接代次并同步 MQTT 会话状态。
 *
 * 作为 esp_at 的连接回调注入。无论开或关都先自增 generation[link]，使所有
 * 属于旧连接的在途/待发报文立即失效。开：新建 MQTT 会话槽；关：释放会话并
 * 停蜂鸣器（该客户端可能正在鸣响）。state_dirty 触发尽快上报客户端数变化。
 */
static void link_changed(uint8_t link, int opened)
{
    char text[48];
    if (link >= MQTT_CLIENTS) { return; }
    ++generation[link];
    tcp_open[link] = opened != 0;
    pending_close[link] = 0U;
    if (opened) { mqtt_endpoint_open(link, board_io_now()); }
    else { mqtt_endpoint_closed(link); board_io_beep(0U); }
    state_dirty = 1U;
    (void)snprintf(text, sizeof(text), "[TCP %u] %s\r\n", (unsigned)link, opened ? "connected" : "closed");
    board_io_log(text);
}

/**
 * @brief 清空网络队列和会话，拉低 ESP8266 复位引脚，重新开始网络初始化。
 */
static void network_reset(void)
{
    memset(tcp_open, 0, sizeof(tcp_open));
    memset(pending_close, 0, sizeof(pending_close));
    queue_head = queue_tail = 0U;
    setup_index = setup_waiting = legacy_command = operation = 0U;
    result_text[0] = '\0';
    mqtt_endpoint_init(enqueue, request_close, handle_command);
    esp_at_init(received_byte, link_changed);
    board_io_beep(0U);
    board_io_wifi_reset(1);
    change_state(APP_RESET);
}

/**
 * @brief 记录网络失败原因，进入等待状态，三秒后重新复位模块。
 */
int app_mqtt_link_command(const char *command, char *reply, size_t capacity)
{
    if (command == NULL || reply == NULL || capacity == 0U) { return 0; }
    if (strncmp(command, "rgb ", 4U) == 0) {
        handle_command(APP_TOPIC_RGB, command + 4U);
    } else if (strncmp(command, "beep ", 5U) == 0) {
        handle_command(APP_TOPIC_BEEP, command + 5U);
    } else if (strcmp(command, "system status") == 0) {
        (void)snprintf(reply, capacity,
                       "app=%u clients=%u rgb=%u beep=%u uptime=%lus commands=%lu",
                       (unsigned)state, mqtt_endpoint_clients(), (unsigned)board_io_rgb_get(),
                       (unsigned)board_io_beep_active(), (unsigned long)(board_io_now()/1000U),
                       (unsigned long)mqtt_endpoint_command_count());
        return 1;
    } else if (strcmp(command, "network restart") == 0) {
        network_reset();
        (void)snprintf(reply, capacity, "ok network restart started (ESP8266 only)");
        return 1;
    } else { return 0; }
    (void)snprintf(reply, capacity, "%s", result_text);
    return 1;
}

static void network_retry(const char *reason)
{
    board_io_log(reason);
    board_io_log("; retry in 3s\r\n");
    board_io_beep(0U);
    change_state(APP_RETRY);
}

/**
 * @brief 按步骤生成 AT 命令：清理旧服务、设置热点和 DHCP，再启动 TCP 监听。
 *
 * 命令序列设计（setup_index 对应 case）：
 *   0 AT            探测模块在线；
 *   1 ATE0          关闭命令回显，简化应答解析；
 *   2 AT+GMR        读取固件版本用于日志排查；
 *   3 AT+CIPSERVER=0 关闭可能残留的旧 TCP 服务器（首次运行时该命令失败属正常）；
 *   4 AT+CIPMUX=0   先切单连接，为配置模式命令提供确定状态；
 *   5 AT+CIPMODE=0  普通 AT 模式，禁用透传；
 *   6 AT+CWMODE=2   SoftAP 热点模式；
 *   7 AT+CWSAP      设置热点 SSID/密码/信道/加密（3=WPA2-PSK）；
 *   8 AT+CIPAP      设置热点静态 IP（网关），子网 255.255.255.0；
 *   9 AT+CWDHCP=0,1 关闭 STA DHCP、开启 AP DHCP，让手机自动获取 IP；
 *   10 AT+CIPDINFO=0 关闭 +IPD 附带远端 IP 信息，保持数据解析格式固定；
 *   11 AT+CIPMUX=1  切回多连接，支持多台手机同时接入；
 *   12 AT+CIPSERVER=1,<port> 在 1883 端口启动 TCP 服务器承载 MQTT；
 *   13 AT+CIPSTO=0  服务器永不因空闲超时踢除连接（心跳由 MQTT 层负责）；
 *   14 AT+CIFSR     回读本机 IP，确认热点配置生效。
 * 新固件命令带 _CUR 后缀（仅改当前配置不写 flash），legacy_command 置位时
 * 退回不带后缀的旧语法。返回 0 表示生成失败（越界或截断）。
 */
static int setup_command(char *command, size_t capacity)
{
    const char *suffix = legacy_command ? "" : "_CUR";
    int length = 0;
    switch (setup_index) {
    case 0: length = snprintf(command, capacity, "AT"); break;
    case 1: length = snprintf(command, capacity, "ATE0"); break;
    case 2: length = snprintf(command, capacity, "AT+GMR"); break;
    case 3: length = snprintf(command, capacity, "AT+CIPSERVER=0"); break;
    case 4: length = snprintf(command, capacity, "AT+CIPMUX=0"); break;
    case 5: length = snprintf(command, capacity, "AT+CIPMODE=0"); break;
    case 6: length = snprintf(command, capacity, "AT+CWMODE%s=2", suffix); break;
    case 7: length = snprintf(command, capacity, "AT+CWSAP%s=\"%s\",\"%s\",%u,3", suffix, APP_WIFI_SSID, APP_WIFI_PASSWORD, (unsigned)APP_WIFI_CHANNEL); break;
    case 8:
        /* 旧版 AT 固件的此命令只接受 IP 地址参数。 */
        if (legacy_command) { length = snprintf(command, capacity, "AT+CIPAP=\"%s\"", APP_WIFI_IP); }
        else { length = snprintf(command, capacity, "AT+CIPAP_CUR=\"%s\",\"%s\",\"255.255.255.0\"", APP_WIFI_IP, APP_WIFI_IP); }
        break;
    case 9: length = snprintf(command, capacity, "AT+CWDHCP%s=0,1", suffix); break;
    case 10: length = snprintf(command, capacity, "AT+CIPDINFO=0"); break;
    case 11: length = snprintf(command, capacity, "AT+CIPMUX=1"); break;
    case 12: length = snprintf(command, capacity, "AT+CIPSERVER=1,%u", (unsigned)APP_MQTT_PORT); break;
    case 13: length = snprintf(command, capacity, "AT+CIPSTO=0"); break;
    case 14: length = snprintf(command, capacity, "AT+CIFSR"); break;
    default: break;
    }
    return length > 0 && (size_t)length < capacity;
}

/**
 * @brief 逐条等待 AT 应答；不支持 _CUR 时尝试旧语法，全部成功后进入就绪状态。
 *
 * 容错策略：
 * - 步骤 3（关旧服务器）与步骤 10（CIPDINFO）失败可容忍：前者在无服务器时
 *   本来就报 ERROR，后者部分固件不支持，均不影响后续配置；
 * - 步骤 6..9 返回 -1（命令错误，通常是固件不认识 _CUR 后缀）时置
 *   legacy_command 重试同一步骤的旧语法，只降级一次；
 * - 其余失败视为环境异常（如热点参数非法），进入 APP_RETRY 三秒后重来。
 */
static void process_setup(void)
{
    /* 仅主循环调用；使用静态缓冲区，减少嵌套调用时的栈占用。 */
    static char command[220];
    static char log[96];
    int result;
    if (setup_waiting) {
        result = esp_at_result();
        if (!result) { return; }
        setup_waiting = 0U;
        if (result < 0 && setup_index != 3U && setup_index != 10U) {
            if (result == -1 && setup_index >= 6U && setup_index <= 9U && !legacy_command) {
                legacy_command = 1U;
                board_io_log("[AT] _CUR unsupported, trying legacy syntax\r\n");
                return;
            }
            (void)snprintf(log, sizeof(log), "[AT] setup step %u/%u failed", (unsigned)(setup_index + 1U), (unsigned)SETUP_COMMANDS);
            network_retry(log);
            return;
        }
        ++setup_index;
        legacy_command = 0U;
    }
    if (setup_index == SETUP_COMMANDS) {
        change_state(APP_READY);
        last_status = board_io_now();
        board_io_log("[READY] WiFi: " APP_WIFI_SSID " | MQTT " APP_WIFI_IP ":1883\r\n");
        board_io_log("[MQTT] v3.1.1, QoS0, Clean Session ON, Retain OFF, no login/will\r\n");
        return;
    }
    if (esp_at_busy()) { return; }
    if (!setup_command(command, sizeof(command))) { change_state(APP_FAILED); return; }
    (void)snprintf(log, sizeof(log), "[AT] setup %u/%u\r\n", (unsigned)(setup_index + 1U), (unsigned)SETUP_COMMANDS);
    board_io_log(log);
    setup_waiting = esp_at_command(command, 8000U) != 0;
}

/**
 * @brief 检查指定连接当前代次是否还有待发报文，以便确定何时可以关闭连接。
 */
static int has_queued(uint8_t link)
{
    uint8_t index = queue_tail;
    while (index != queue_head) {
        if (queue[index].link == link && queue[index].generation == generation[link]) { return 1; }
        index = (uint8_t)((index + 1U) % QUEUE_COUNT);
    }
    return 0;
}

/**
 * @brief 串行执行发送和关闭事务，消费 AT 完成结果，并丢弃失效连接的旧报文。
 *
 * ESP8266 的 AT 接口同一时刻只能进行一个事务（命令或 CIPSEND），因此用
 * operation 变量记录当前在途事务：1=CIPSEND 数据发送，2=CIPCLOSE 关闭连接。
 * 事务结果由 esp_at_result() 一次性消费；事务完成前不再启动新事务。
 * 出队时校验 link 仍打开且代次未变，否则说明报文属于已失效的连接，直接跳过。
 * 关闭请求（pending_close）必须等该连接当前代次的待发报文全部处理完才执行，
 * 保证 MQTT 应答（如 CONNACK）先于 TCP 断开送达对端。
 */
static void process_network_output(void)
{
    int result;
    unsigned i;
    queued_packet_t *packet;
    if (operation) {
        result = esp_at_result();
        if (!result) { return; }
        if (operation_generation == generation[operation_link]) {
            if (operation == 1U && result < 0) {
                /* 发送失败可能导致对端保留不完整 MQTT 报文，因此关闭会话重新连接。 */
                pending_close[operation_link] = 1U;
                ++generation[operation_link];
                mqtt_endpoint_closed(operation_link);
            } else if (operation == 2U) {
                tcp_open[operation_link] = 0U;
                pending_close[operation_link] = 0U;
                ++generation[operation_link];
                mqtt_endpoint_closed(operation_link);
            }
        }
        operation = 0U;
    }
    if (esp_at_busy()) { return; }
    while (queue_tail != queue_head) {
        packet = &queue[queue_tail];
        if (!tcp_open[packet->link] || packet->generation != generation[packet->link]) {
            queue_tail = (uint8_t)((queue_tail + 1U) % QUEUE_COUNT);
            continue;
        }
        if (!esp_at_send(packet->link, packet->bytes, packet->length)) { return; }
        operation = 1U;
        operation_link = packet->link;
        operation_generation = packet->generation;
        queue_tail = (uint8_t)((queue_tail + 1U) % QUEUE_COUNT);
        return;
    }
    for (i = 0U; i < MQTT_CLIENTS; ++i) {
        if (pending_close[i] && tcp_open[i] && !has_queued((uint8_t)i) && esp_at_close((uint8_t)i)) {
            operation = 2U;
            operation_link = (uint8_t)i;
            operation_generation = generation[i];
            return;
        }
    }
}

/**
 * @brief 生成运行时间、灯光、蜂鸣器、客户端数量和命令计数的 JSON 状态消息。
 */
static void publish_state(void)
{
    char text[180];
    char distance[32];
    (void)snprintf(text, sizeof(text), "{\"uptime\":%lu,\"rgb\":%u,\"beep\":%u,\"clients\":%u,\"commands\":%lu}",
                   (unsigned long)(board_io_now() / 1000U), (unsigned)board_io_rgb_get(),
                   (unsigned)board_io_beep_active(), mqtt_endpoint_clients(),
                   (unsigned long)mqtt_endpoint_command_count());
    mqtt_endpoint_publish(APP_TOPIC_STATE, text);
    /* 随状态周期上报距离，单位厘米；错误状态也上报，避免手机保留失效距离。 */
    if (app_hcsr04_format(distance, sizeof(distance))) {
        mqtt_endpoint_publish(APP_TOPIC_DISTANCE, distance);
    }
}

/**
 * @brief 解析一行调试串口命令；舵机命令示例为 servo 0 90，需以换行结束。
 */
static void console_command(void)
{
    char text[160];
    console_line[console_length] = '\0';
    if (strcmp(console_line, "status") == 0) {
        (void)snprintf(text, sizeof(text), "[STATUS] app=%u clients=%u rgb=%u beep=%u uptime=%lus\r\n",
                       (unsigned)state, mqtt_endpoint_clients(), (unsigned)board_io_rgb_get(),
                       (unsigned)board_io_beep_active(), (unsigned long)(board_io_now() / 1000U));
        board_io_log(text);
    } else if (strcmp(console_line, "restart") == 0) { network_reset(); }
    else if (strncmp(console_line, "rgb ", 4U) == 0) { handle_command(APP_TOPIC_RGB, console_line + 4U); }
    else if (strncmp(console_line, "beep ", 5U) == 0) { handle_command(APP_TOPIC_BEEP, console_line + 5U); }
    else if (strncmp(console_line, "gait ", 5U) == 0) { handle_command(APP_TOPIC_SERVO, console_line); }
    else if (strncmp(console_line, "servo ", 6U) == 0) { handle_command(APP_TOPIC_SERVO, console_line + 6U); }
    else if (console_length) { board_io_log("Commands: status | restart | rgb red | rgb off | beep 100 | beep off\r\n"); }
    console_length = 0U;
}

/**
 * @brief 限量读取串口字符；支持退格，过长命令丢弃到下一次换行，防止缓冲区溢出。
 */
static void process_console(void)
{
    uint8_t byte;
    unsigned budget = 128U;
    while (budget-- && board_io_console_read(&byte)) {
        if (byte == '\r') { continue; }
        if (byte == '\n') {
            if (!console_discard) { console_command(); }
            console_discard = 0;
            console_length = 0U;
        } else if (byte == 8U || byte == 127U) { if (console_length) { --console_length; } }
        else if (byte >= 32U && byte <= 126U && !console_discard) {
            if (console_length + 1U < sizeof(console_line)) { console_line[console_length++] = (char)byte; }
            else { console_discard = 1; }
        }
    }
}

/**
 * @brief 初始化板级接口、舵机应用和网络状态；应在 CubeMX 外设初始化后调用。
 */
void app_mqtt_init(void)
{
    if (!board_io_init()) { state = APP_FAILED; return; }
    board_io_log("\r\nGuide MQTT AP endpoint | STM32F103VET6 | USART3 115200\r\n");
    board_io_log("Serial console: status / restart / rgb red / beep 100 (newline required)\r\n");
    network_reset();
}

/**
 * @brief 主循环周期调用：推进网络状态机、处理控制命令，并按需或每两秒发布状态。
 */
void app_mqtt_process(void)
{
    uint32_t elapsed;
    char servo_reply[80];
    char command_log[112];
    app_hcsr04_report();
    board_io_process();
    process_console();
    if (state == APP_SETUP || state == APP_READY) { esp_at_process(); }
    /* AT 传输一旦故障（串口溢出、超时或模块重启），非终态下统一转入重试。 */
    if (esp_at_fault() && state != APP_RETRY && state != APP_FAILED) { network_retry("[AT] transport lost/timeout"); }
    /* 无符号差值计时，天然兼容 32 位毫秒计数约 49.7 天的回绕。 */
    elapsed = (uint32_t)(board_io_now() - state_started);
    switch (state) {
    case APP_RESET:
        /* 复位脚保持低电平 100ms 后释放，满足 ESP8266 最小复位脉宽。 */
        if (elapsed >= 100U) { board_io_wifi_reset(0); change_state(APP_BOOT); }
        break;
    case APP_BOOT:
        /* 释放复位后等待 2s 让模块完成内部启动，再清缓冲并开始 AT 握手。 */
        if (elapsed >= 2000U) {
            board_io_wifi_prepare_rx();
            esp_at_init(received_byte, link_changed);
            change_state(APP_SETUP);
        }
        break;
    case APP_SETUP: process_setup(); break;
    case APP_READY:
        mqtt_endpoint_process(board_io_now());   /* 驱动 MQTT 会话超时检测 */
        process_network_output();                /* 串行处理发送/关闭事务 */
        /* 状态变化立即上报，否则每 2s 周期上报一次心跳状态。 */
        if (state_dirty || (uint32_t)(board_io_now() - last_status) >= 2000U) {
            publish_state();
            last_status = board_io_now();
            state_dirty = 0U;
        }
        break;
    case APP_RETRY:
        /* 故障后延时 3s，让对端/模块稳定，再整体复位重来。 */
        if (elapsed >= 3000U) { network_reset(); }
        break;
    default: break;
    }
    if (result_text[0]) {
        (void)snprintf(command_log, sizeof(command_log), "[COMMAND] %s\r\n", result_text);
        board_io_log(command_log);
        if (state == APP_READY) { mqtt_endpoint_publish(APP_TOPIC_RESULT, result_text); }
        result_text[0] = '\0';
    }
    if (app_rtos_servo_result(servo_reply, sizeof(servo_reply))) {
        (void)snprintf(command_log, sizeof(command_log), "[COMMAND] %s\r\n", servo_reply);
        board_io_log(command_log);
        if (state == APP_READY) { mqtt_endpoint_publish(APP_TOPIC_RESULT, servo_reply); }
    }
}
