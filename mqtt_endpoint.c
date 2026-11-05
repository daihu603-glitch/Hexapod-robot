/**
 * @file mqtt_endpoint.c
 * @brief 本地 MQTT 服务端：维护有限个客户端和订阅，执行控制命令并转发匹配主题。
 *
 * 在系统中的角色：STM32 借助 ESP8266 SoftAP + TCP 服务器（1883 端口）自建
 * 一个轻量 MQTT 代理（broker），手机作为 MQTT 客户端接入。本模块实现服务端
 * 协议逻辑：CONNECT/SUBSCRIBE/PUBLISH/PINGREQ/DISCONNECT 处理、订阅表管理、
 * 主题通配匹配和会话超时踢除；业务命令经 on_command 回调交给 app_mqtt。
 * 所有报文收发都不直接碰串口，而是通过注入的 send/close 回调走 app_mqtt
 * 的发送队列，保持"传输层串行、协议层纯逻辑"的分层。
 *
 * 设计约束（面向本地局域网、资源受限的 STM32F103）：
 * - 客户端数上限 MQTT_CLIENTS，每客户端订阅槽 SUBSCRIPTION_COUNT 个；
 * - 只支持 QoS0、Clean Session，拒绝遗嘱/认证/保留消息；
 * - 主题与载荷限定为可打印 ASCII，便于字符串比较和日志输出。
 */
#include "mqtt_endpoint.h"
#include "mqtt_codec.h"
#include <string.h>

#define SUBSCRIPTION_COUNT 4U   /* 每客户端订阅过滤器槽位数 */
#define TOPIC_CAPACITY 80U      /* 主题字符串上限（含 '\0'） */
#define CLIENT_ID_CAPACITY 64U  /* Client ID 字符串上限（含 '\0'） */
#define COMMAND_CAPACITY 96U    /* 命令载荷字符串上限（含 '\0'） */

typedef struct {
    mqtt_stream_t stream;        /* 该连接的字节流组包器 */
    uint8_t open;                /* TCP 已建立（等待或已完成 MQTT CONNECT） */
    uint8_t connected;           /* MQTT 会话已建立（CONNACK 已发送） */
    uint16_t keepalive;          /* CONNECT 声明的保活秒数，0 表示不检查 */
    uint32_t last_packet;        /* 最近收到完整报文的时刻，用于保活超时 */
    uint32_t fragment_started;   /* 当前半包开始累积的时刻，用于分包超时 */
    char id[CLIENT_ID_CAPACITY];
    char subscriptions[SUBSCRIPTION_COUNT][TOPIC_CAPACITY]; /* 空串表示槽位空闲 */
} client_t;

static client_t clients[MQTT_CLIENTS];
static mqtt_send_fn send_packet;    /* 报文入发送队列（app_mqtt 的 enqueue） */
static mqtt_close_fn close_link;    /* 申请关闭 TCP（app_mqtt 的 request_close） */
static mqtt_command_fn on_command;  /* 业务命令回调（app_mqtt 的 handle_command） */
static uint8_t outgoing[MQTT_PACKET_CAPACITY]; /* 发布报文公共组包缓冲，仅主循环使用 */
static uint32_t command_count;      /* 通过校验的 PUBLISH 计数，供状态上报 */

/**
 * @brief 将会话标记为断开，并通过回调申请关闭底层 TCP 连接。
 */
static void disconnect(uint8_t link)
{
    clients[link].open = 0U;
    clients[link].connected = 0U;
    if (close_link) { close_link(link); }
}

/**
 * @brief 经上层队列发送 MQTT 报文；无法入队时断开该客户端。
 */
static int send_to(uint8_t link, const uint8_t *packet, size_t length)
{
    if (!send_packet || !send_packet(link, packet, length)) { disconnect(link); return 0; }
    return 1;
}

/**
 * @brief 清空客户端和命令计数，注册发送、关闭连接及业务命令回调。
 */
void mqtt_endpoint_init(mqtt_send_fn send, mqtt_close_fn close, mqtt_command_fn command)
{
    memset(clients, 0, sizeof(clients));
    send_packet = send;
    close_link = close;
    on_command = command;
    command_count = 0U;
}

/**
 * @brief TCP 连接建立时清空对应槽位，并开始等待客户端 CONNECT 报文。
 */
void mqtt_endpoint_open(uint8_t link, uint32_t now)
{
    if (link >= MQTT_CLIENTS) { return; }
    memset(&clients[link], 0, sizeof(clients[link]));
    clients[link].open = 1U;
    clients[link].last_packet = now;
}

/**
 * @brief TCP 连接断开后释放该槽位的会话、订阅和组包状态。
 */
void mqtt_endpoint_closed(uint8_t link)
{
    if (link < MQTT_CLIENTS) { memset(&clients[link], 0, sizeof(clients[link])); }
}

/* 本地服务的主题和客户端标识限定为有长度上限的可打印 ASCII 字符串。 */
/**
 * @brief 读取带双字节长度的字符串；当前服务限定为可打印 ASCII，并校验输出容量。
 */
static int read_string(const uint8_t *body, size_t length, size_t *offset, char *out, size_t capacity)
{
    size_t count;
    size_t i;
    if (*offset > length || length - *offset < 2U) { return 0; }
    count = ((size_t)body[*offset] << 8) | body[*offset + 1U];
    *offset += 2U;
    if (count >= capacity || count > length - *offset) { return 0; }
    for (i = 0U; i < count; ++i) {
        if (body[*offset + i] < 32U || body[*offset + i] > 126U) { return 0; }
        out[i] = (char)body[*offset + i];
    }
    out[count] = '\0';
    *offset += count;
    return 1;
}

/**
 * @brief 校验 MQTT 3.1.1 和连接选项，回复 CONNACK；相同 Client ID 的旧会话会被断开。
 *
 * CONNECT 可变头固定 10 字节：协议名(6) + 级别(1) + 标志(1) + KeepAlive(2)，
 * 因此载荷从 offset=10 开始，且载荷必须恰好只有一个 Client ID 字符串
 * （offset 最终须等于 length，否则说明带了遗嘱/认证等被禁止的字段）。
 * CONNACK 4 字节 {0x20, 2, 会话呈现标志 0, 返回码}，返回码含义（规范 3.2.2.3）：
 *   0 接受；1 协议版本不支持（级别 != 4）；2 Client ID 非法（空串/非 ASCII/
 *   超长/夹带多余字段）；5 未授权（连接标志 != 0x02，即不是纯 Clean Session）。
 * 拒绝时也先发 CONNACK 再断开，让客户端能看到原因。
 * 顶替规则：新连接使用已存在的 Client ID 时，按 MQTT 惯例断开旧会话，
 * 保证同一 ID 全局只有一个活动客户端。
 */
static void handle_connect(uint8_t link, const uint8_t *body, size_t length)
{
    uint8_t reply[4] = {0x20U, 2U, 0U, 0U};
    size_t offset = 10U;
    unsigned i;
    client_t *client = &clients[link];
    if (client->connected || length < 12U || memcmp(body, "\0\4MQTT", 6U) != 0) { disconnect(link); return; }
    if (body[6] != 4U) { reply[3] = 1U; }
    /* 本地服务只接受清理会话，不接受遗嘱、用户名或密码字段。 */
    else if (body[7] != 2U) { reply[3] = 5U; }
    else if (!read_string(body, length, &offset, client->id, sizeof(client->id)) || !client->id[0] || offset != length) { reply[3] = 2U; }
    if (reply[3]) {
        (void)send_to(link, reply, sizeof(reply));
        disconnect(link);
        return;
    }
    for (i = 0U; i < MQTT_CLIENTS; ++i) {
        if (i != link && clients[i].connected && strcmp(clients[i].id, client->id) == 0) { disconnect((uint8_t)i); }
    }
    client->keepalive = (uint16_t)(((uint16_t)body[8] << 8) | body[9]);
    client->connected = 1U;
    (void)send_to(link, reply, sizeof(reply));
}

/**
 * @brief 校验订阅过滤器：# 只能占据最后一级，+ 必须单独占据一级。
 *
 * 通配规则（规范 4.7）：'#' 多层通配，必须是过滤器最后一个字符且前面紧邻
 * '/'（或整个过滤器就是 "#"）；'+' 单层通配，必须完整占据一级——要么在
 * 开头，要么前面是 '/'，且后面要么是 '/' 要么是字符串结束。任何违反都
 * 返回 0，上层据此断开客户端，防止非法过滤器进入订阅表。
 */
static int filter_valid(const char *filter)
{
    size_t i;
    if (!filter[0]) { return 0; }
    for (i = 0U; filter[i]; ++i) {
        if (filter[i] == '#' && ((i && filter[i - 1U] != '/') || filter[i + 1U])) { return 0; }
        if (filter[i] == '+' && ((i && filter[i - 1U] != '/') || (filter[i + 1U] && filter[i + 1U] != '/'))) { return 0; }
    }
    return 1;
}

/**
 * @brief 逐层匹配订阅过滤器和主题，支持 + 与 #，并遵守 $ 主题的通配匹配限制。
 *
 * 匹配算法按 '/' 分隔逐级推进：
 * - 过滤器级为 '#'：直接匹配剩余全部层级（含零层），返回 1；
 * - 过滤器级为 '+'：跳过主题的当前一整级（到 '/' 或结束为止）；
 * - 普通级：逐字符比较，任一字符不等即失败；
 * - 每级结束后双方都耗尽则匹配成功；特殊情形：主题已耗尽而过滤器恰为
 *   "/#"（如 "sport/#" 应匹配 "sport" 本身，规范 4.7.1.2）也判成功；
 * - 一方还有层级而另一方没有、或分隔符不对齐，判失败。
 * '$' 开头的系统主题（如 $SYS）不允许被 '#'/'+' 开头的过滤器捕获（规范 4.7.2）。
 */
static int topic_matches(const char *filter, const char *topic)
{
    const char *f = filter;
    const char *t = topic;
    if (*t == '$' && (*f == '#' || *f == '+')) { return 0; }
    for (;;) {
        if (*f == '#') { return 1; }
        if (*f == '+') { while (*t && *t != '/') { ++t; } ++f; }
        else { while (*f && *f != '/' && *t && *t != '/') { if (*f++ != *t++) { return 0; } } }
        if (!*f && !*t) { return 1; }
        if (!*t && strcmp(f, "/#") == 0) { return 1; }
        if (*f != '/' || *t != '/') { return 0; }
        ++f;
        ++t;
    }
}

/**
 * @brief 优先返回已有订阅位置，否则返回第一个空槽；无可用槽返回 -1。
 */
static int subscription_slot(client_t *client, const char *topic)
{
    unsigned i;
    int empty = -1;
    for (i = 0U; i < SUBSCRIPTION_COUNT; ++i) {
        if (strcmp(client->subscriptions[i], topic) == 0) { return (int)i; }
        if (!client->subscriptions[i][0] && empty < 0) { empty = (int)i; }
    }
    return empty;
}

/**
 * @brief 处理订阅或取消订阅，并按报文标识符回复 SUBACK 或 UNSUBACK。
 *
 * SUBSCRIBE(0x82)/UNSUBSCRIBE(0xA2) 载荷结构：双字节报文标识符 +
 * 一个或多个"主题过滤器"（SUBSCRIBE 每个主题后还跟一个请求 QoS 字节）。
 * 应答：SUBACK(0x90) 回显标识符并给每个主题一个返回码（0x00 授予 QoS0，
 * 0x80 失败=订阅槽已满）；UNSUBACK(0xB0) 只回显标识符。
 * 边界检查：报文标识符为 0 违反规范；单个报文的主题数超过槽位总数、
 * 过滤器非法、请求 QoS > 2 都直接断开——本地服务选择严格而非宽容，
 * 因为对接的是可信的手机客户端。重复订阅同一主题会命中原槽位，不浪费槽。
 */
static void handle_subscribe(uint8_t link, const uint8_t *body, size_t length, int unsubscribe)
{
    size_t offset = 2U;
    size_t count = 0U;
    uint8_t reply[4U + SUBSCRIPTION_COUNT];
    char topic[TOPIC_CAPACITY];
    int slot;
    unsigned i;
    if (length < 5U || (!body[0] && !body[1])) { disconnect(link); return; }
    reply[0] = unsubscribe ? 0xB0U : 0x90U;
    reply[2] = body[0];
    reply[3] = body[1];
    while (offset < length) {
        if (count >= SUBSCRIPTION_COUNT || !read_string(body, length, &offset, topic, sizeof(topic)) || !filter_valid(topic)) { disconnect(link); return; }
        if (unsubscribe) {
            for (i = 0U; i < SUBSCRIPTION_COUNT; ++i) {
                if (strcmp(clients[link].subscriptions[i], topic) == 0) { clients[link].subscriptions[i][0] = '\0'; }
            }
        } else {
            if (offset >= length || body[offset++] > 2U) { disconnect(link); return; }
            slot = subscription_slot(&clients[link], topic);
            reply[4U + count] = slot >= 0 ? 0U : 0x80U;
            if (slot >= 0) { strcpy(clients[link].subscriptions[slot], topic); }
        }
        ++count;
    }
    if (!count) { disconnect(link); return; }
    reply[1] = unsubscribe ? 2U : (uint8_t)(2U + count);
    (void)send_to(link, reply, unsubscribe ? 4U : 4U + count);
}

/**
 * @brief 生成无保留的 QoS0 消息，向所有匹配订阅的已连接客户端各发送一次。
 */
void mqtt_endpoint_publish(const char *topic, const char *payload)
{
    unsigned i;
    unsigned sub;
    size_t length = mqtt_publish(outgoing, sizeof(outgoing), topic, payload, 0);
    if (!length) { return; }
    for (i = 0U; i < MQTT_CLIENTS; ++i) {
        if (!clients[i].connected) { continue; }
        for (sub = 0U; sub < SUBSCRIPTION_COUNT; ++sub) {
            if (clients[i].subscriptions[sub][0] && topic_matches(clients[i].subscriptions[sub], topic)) {
                (void)send_to((uint8_t)i, outgoing, length);
                break;
            }
        }
    }
}

/**
 * @brief 校验 QoS0、非保留和 ASCII 载荷后调用业务回调，再将消息转发给订阅者。
 *
 * 服务端策略：只接受 QoS0、非 RETAIN 的 PUBLISH（本机资源受限，不维护
 * QoS1/2 重传状态机与保留消息存储），违规一律断开。主题额外禁止含
 * '+'/'#'（规范规定 PUBLISH 主题必须是具体主题，不允许通配符）。
 * 校验通过后把主题/载荷复制为 C 字符串（回调接口按字符串设计），先交
 * on_command 执行业务（舵机/RGB/蜂鸣器），再把消息原样转发给所有订阅了
 * 该主题的客户端——包括发送者自己，实现多端状态同步。
 */
static void handle_publish(uint8_t link, const uint8_t *packet, size_t length)
{
    mqtt_publish_t message;
    char topic[TOPIC_CAPACITY];
    char payload[COMMAND_CAPACITY];
    size_t i;
    if (!mqtt_read_publish(packet, length, &message) || message.qos || message.retained) { disconnect(link); return; }
    if (message.topic_length >= sizeof(topic) || message.payload_length >= sizeof(payload)) { disconnect(link); return; }
    for (i = 0U; i < message.topic_length; ++i) {
        uint8_t c = message.topic[i];
        if (c < 32U || c > 126U || c == '+' || c == '#') { disconnect(link); return; }
    }
    for (i = 0U; i < message.payload_length; ++i) {
        if (message.payload[i] < 32U || message.payload[i] > 126U) { disconnect(link); return; }
    }
    memcpy(topic, message.topic, message.topic_length);
    topic[message.topic_length] = '\0';
    memcpy(payload, message.payload, message.payload_length);
    payload[message.payload_length] = '\0';
    ++command_count;
    if (on_command) { on_command(topic, payload); }
    mqtt_endpoint_publish(topic, payload);
}

/**
 * @brief 按固定头分发连接、发布、订阅、取消订阅、心跳和断开报文。
 *
 * 报文类型 = 固定头字节高 4 位（左移后与低 4 位标志合并判断）：
 * 0x10 CONNECT（仅未建会话时受理）；0x30 PUBLISH；0x82 SUBSCRIBE；
 * 0xA2 UNSUBSCRIBE；0xC0 PINGREQ（可变头必须为空，回 0xD0 PINGRESP）；
 * 0xE0 DISCONNECT（主动断开，清会话并关 TCP）。
 * 未建会话的连接除 CONNECT 外任何报文都断开；未知类型同样断开。
 */
static void handle_packet(uint8_t link)
{
    client_t *client = &clients[link];
    const uint8_t *packet = client->stream.bytes;
    const uint8_t *body;
    size_t length;
    const uint8_t pong[2] = {0xD0U, 0U};
    if (!mqtt_packet_body(packet, client->stream.length, &body, &length)) { disconnect(link); return; }
    if (packet[0] == 0x10U) { handle_connect(link, body, length); return; }
    if (!client->connected) { disconnect(link); return; }
    switch (packet[0]) {
    case 0x30U: handle_publish(link, packet, client->stream.length); break;
    case 0x82U: handle_subscribe(link, body, length, 0); break;
    case 0xA2U: handle_subscribe(link, body, length, 1); break;
    case 0xC0U:
        if (length) { disconnect(link); break; }
        (void)send_to(link, pong, sizeof(pong));
        break;
    case 0xE0U: disconnect(link); break;
    default: disconnect(link); break;
    }
}

/**
 * @brief 按连接逐字节组包；收齐后处理并重置组包器，非法报文触发断开。
 */
void mqtt_endpoint_feed(uint8_t link, uint8_t byte, uint32_t now)
{
    int result;
    client_t *client;
    if (link >= MQTT_CLIENTS || !clients[link].open) { return; }
    client = &clients[link];
    if (!client->stream.length) { client->fragment_started = now; }
    result = mqtt_stream_feed(&client->stream, byte);
    if (result < 0) { disconnect(link); return; }
    if (result > 0) {
        client->last_packet = now;
        handle_packet(link);
        mqtt_stream_reset(&client->stream);
    }
}

/**
 * @brief 检查超时：CONNECT 等待 10 秒、分包等待 5 秒、心跳等待 Keep Alive 的 1.5 倍。
 *
 * 超时策略说明（由 app_mqtt 主循环周期驱动）：
 * - 已建会话：允许静默 keepalive*1500 毫秒（keepalive 秒 × 1.5，规范 3.1.2.10
 *   建议服务端在 1.5 倍保活期无报文后断开；keepalive=0 时 limit 为 0，跳过检查）；
 * - 未建会话（TCP 已通但迟迟不发 CONNECT）：10 秒内必须完成握手，防止
 *   端口扫描或非 MQTT 客户端长期占用有限的会话槽位；
 * - 半包超时：报文收到一半后 5 秒内没收齐则断开，防止慢速/恶意客户端用
 *   残缺报文长期霸占组包缓冲。
 * 所有计时均用无符号毫秒差值，兼容计数器回绕。超时一律断开并释放 TCP。
 */
void mqtt_endpoint_process(uint32_t now)
{
    unsigned i;
    for (i = 0U; i < MQTT_CLIENTS; ++i) {
        client_t *client = &clients[i];
        uint32_t limit = client->connected ? (uint32_t)client->keepalive * 1500U : 10000U;
        if (!client->open) { continue; }
        if ((limit && (uint32_t)(now - client->last_packet) > limit) ||
            (client->stream.length && (uint32_t)(now - client->fragment_started) > 5000U)) { disconnect((uint8_t)i); }
    }
}

/**
 * @brief 统计完成 MQTT CONNECT 握手的客户端数量。
 */
unsigned mqtt_endpoint_clients(void)
{
    unsigned i;
    unsigned count = 0U;
    for (i = 0U; i < MQTT_CLIENTS; ++i) { if (clients[i].connected) { ++count; } }
    return count;
}

/**
 * @brief 返回本次服务初始化以来通过校验的发布消息数量。
 */
uint32_t mqtt_endpoint_command_count(void) { return command_count; }
