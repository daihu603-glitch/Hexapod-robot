/**
 * @file mqtt_codec.c
 * @brief MQTT 报文编解码：构造网络字节序数据，按 Remaining Length 从 TCP 字节流恢复报文。
 *
 * 在系统中的角色：STM32 本地 MQTT 服务端（mqtt_endpoint）的纯编解码库，
 * 不依赖任何硬件接口，负责两件事：
 * 1) 编码：构造 CONNACK 之外的客户端方向报文（CONNECT/SUBSCRIBE/PUBLISH），
 *    当前主要用于测试与通用性保留，服务端运行时以构造 PUBLISH 为主；
 * 2) 解码：流式组包器 mqtt_stream_feed 按 MQTT 3.1.1 的固定头 +
 *    变长 Remaining Length 从 TCP 字节流中切分出完整报文，
 *    mqtt_read_publish 再解析 PUBLISH 的主题/载荷。
 *
 * MQTT 3.1.1 报文结构（规范第 2 节）：
 *   字节 0：固定头 = 报文类型（高 4 位）| 类型专属标志（低 4 位）；
 *   字节 1..4：Remaining Length 变长整数，每字节低 7 位有效、最高位 0x80
 *   为"续接位"，最多 4 字节，可表示 0..268435455；其后是可变头与载荷。
 * 多字节整数（长度、报文标识符等）一律大端（网络字节序）。
 */
#include "mqtt_codec.h"
#include <string.h>

/* 报文格式依据 OASIS MQTT 3.1.1 第 2.2、3.1、3.3、3.8 节。 */
typedef struct { uint8_t *data; size_t capacity; size_t length; int valid; } writer_t;

/**
 * @brief 向有界写入器追加一个字节；空间不足时置无效标志，后续构包返回失败。
 */
static void put_byte(writer_t *writer, uint8_t value)
{
    if (!writer->valid || writer->length >= writer->capacity) { writer->valid = 0; return; }
    writer->data[writer->length++] = value;
}

/**
 * @brief 以高字节在前的网络字节序写入 16 位数。
 */
static void put_u16(writer_t *writer, uint16_t value)
{
    put_byte(writer, (uint8_t)(value >> 8));
    put_byte(writer, (uint8_t)value);
}

/**
 * @brief 先写入双字节长度，再写字符串内容，不把字符串结尾的空字符放入报文。
 */
static void put_string(writer_t *writer, const char *value)
{
    size_t length = strlen(value);
    size_t i;
    if (length > 65535U) { writer->valid = 0; return; }
    put_u16(writer, (uint16_t)length);
    for (i = 0U; i < length && writer->valid; ++i) { put_byte(writer, (uint8_t)value[i]); }
}

/**
 * @brief 将预留的五字节头部压缩为固定头和变长长度字段；返回最终报文长度，失败返回 0。
 *
 * 编码技巧：所有 put_* 都从 data[5] 开始写报文体，前 5 字节（1 固定头 +
 * 最多 4 长度字节）先留白。等报文体长度 remaining 确定后，才能算出变长
 * Remaining Length 实际占用几个字节 count，再用 memmove 把体前移到
 * data[1+count]，最后回填 data[0]=header 与 data[1..count]=encoded。
 * 变长编码：每字节写 value%128，若还有高位则置续接位 0x80，循环到值为 0；
 * 4 字节仍放不下（value 非 0）说明报文过大，返回 0 表示失败。
 */
static size_t finish(writer_t *writer, uint8_t header)
{
    uint8_t encoded[4];
    size_t count = 0U;
    size_t remaining = writer->length - 5U;
    size_t value = remaining;
    if (!writer->valid) { return 0U; }
    do {
        encoded[count] = (uint8_t)(value % 128U);
        value /= 128U;
        if (value) { encoded[count] |= 0x80U; }
        ++count;
    } while (value && count < sizeof(encoded));
    if (value) { return 0U; }
    memmove(writer->data + 1U + count, writer->data + 5U, remaining);
    writer->data[0] = header;
    memcpy(writer->data + 1U, encoded, count);
    return 1U + count + remaining;
}

/**
 * @brief 构造 MQTT 3.1.1 CONNECT；包含清理会话和 offline 遗嘱，按非空参数附加认证字段。
 * 这是通用构包接口，当前本地服务端只接受无认证、无遗嘱的连接配置。
 *
 * CONNECT 可变头顺序（规范 3.1）：协议名 "MQTT"（带双字节长度）、协议级别
 * 4（=3.1.1）、连接标志 flags、Keep Alive（秒），随后是载荷：Client ID、
 * 遗嘱主题、遗嘱消息，以及可选的用户名/密码。
 * flags=0x26 = Clean Session(0x02) | Will Flag(0x04) | Will QoS0 | Will Retain(0x20)。
 * 用户名/密码非空时按规范置 0x80/0x40 并追加对应字段。固定头 0x10 = CONNECT。
 * 参数含空 Client ID/空遗嘱主题时返回 0（本地策略要求二者非空）。
 */
size_t mqtt_connect(uint8_t *out, size_t capacity, const char *client_id,
                    const char *username, const char *password,
                    const char *will_topic, uint16_t keepalive)
{
    writer_t writer = {out, capacity, 5U, 1};
    uint8_t flags = 0x26U; /* 清理会话，加带保留位的 QoS0 离线遗嘱。 */
    if (!out || capacity < 5U || !client_id || !*client_id || !username || !password || !will_topic || !*will_topic) { return 0U; }
    if (*username) { flags |= 0x80U; }
    if (*password) { flags |= 0x40U; }
    put_string(&writer, "MQTT");
    put_byte(&writer, 4U);
    put_byte(&writer, flags);
    put_u16(&writer, keepalive);
    put_string(&writer, client_id);
    put_string(&writer, will_topic);
    put_string(&writer, "offline");
    if (*username) { put_string(&writer, username); }
    if (*password) { put_string(&writer, password); }
    return finish(&writer, 0x10U);
}

/**
 * @brief 构造 SUBSCRIBE，报文标识符 id 必须非零，申请的订阅服务质量为 QoS0。
 * 固定头 0x82 = SUBSCRIBE 类型(8)<<4 | 规范要求的保留位 0x02。
 * 可变头为双字节报文标识符，载荷为"主题过滤器 + 请求 QoS 字节"序列。
 */
size_t mqtt_subscribe(uint8_t *out, size_t capacity, const char *topic, uint16_t id)
{
    writer_t writer = {out, capacity, 5U, 1};
    if (!out || capacity < 5U || !topic || !*topic || !id) { return 0U; }
    put_u16(&writer, id);
    put_string(&writer, topic);
    put_byte(&writer, 0U); /* 请求 QoS0 订阅。 */
    return finish(&writer, 0x82U);
}

/**
 * @brief 构造 QoS0 PUBLISH；retained 非零时设置保留位，成功返回长度，失败返回 0。
 * 固定头 0x30 = PUBLISH 类型(3)<<4，QoS0 无报文标识符；保留位 bit0 即 RETAIN。
 * 载荷按 C 字符串逐字节写入（不含结尾 '\0'）。
 */
size_t mqtt_publish(uint8_t *out, size_t capacity, const char *topic, const char *payload, int retained)
{
    writer_t writer = {out, capacity, 5U, 1};
    size_t i;
    if (!out || capacity < 5U || !topic || !*topic || !payload) { return 0U; }
    put_string(&writer, topic);
    for (i = 0U; payload[i] && writer.valid; ++i) { put_byte(&writer, (uint8_t)payload[i]); }
    return finish(&writer, retained ? 0x31U : 0x30U);
}

/**
 * @brief 清空流式组包长度；完整报文消费后必须调用，才能接收下一包。
 */
void mqtt_stream_reset(mqtt_stream_t *stream)
{
    if (!stream) { return; }
    stream->length = 0U;
    stream->expected = 0U;
}

/**
 * @brief 追加一个字节并解析变长长度：1 为完整报文，0 为未收齐，-1 为非法或超长。
 *
 * 流式组包原理：TCP 是字节流，MQTT 报文可能跨多个 +IPD 段到达。本函数
 * 每收到一字节就存入 stream->bytes；当长度字段尚未确定（expected==0）且
 * 已收到 2 字节以上时，从 bytes[1] 起按 7 位分组 + 0x80 续接位规则重算
 * Remaining Length。确定后 expected = 头部字节数 + remaining，作为整包
 * 期望长度；此后只需比较 length == expected 即可判断报文收齐。
 * 非法判定：长度字段超过 4 字节、remaining 超出缓冲区剩余空间、
 * 或累积总长达到缓冲上限，都返回 -1，由上层断开连接（防恶意超大报文）。
 */
int mqtt_stream_feed(mqtt_stream_t *stream, uint8_t byte)
{
    size_t i;
    size_t remaining = 0U;
    size_t multiplier = 1U;
    if (!stream || stream->length >= sizeof(stream->bytes)) { return -1; }
    stream->bytes[stream->length++] = byte;
    if (!stream->expected && stream->length > 1U) {
        for (i = 1U; i < stream->length; ++i) {
            if (i > 4U) { return -1; }
            remaining += (size_t)(stream->bytes[i] & 127U) * multiplier;
            if (!(stream->bytes[i] & 128U)) {
                if (remaining > sizeof(stream->bytes) - i - 1U) { return -1; }
                stream->expected = i + 1U + remaining;
                break;
            }
            if (i == 4U) { return -1; }
            multiplier *= 128U;
        }
    }
    return stream->expected && stream->length == stream->expected ? 1 : 0;
}

/**
 * @brief 校验长度字段与实际报文一致，返回报文体指针和字节数。
 */
int mqtt_packet_body(const uint8_t *packet, size_t length, const uint8_t **body, size_t *body_length)
{
    size_t i;
    size_t remaining = 0U;
    size_t multiplier = 1U;
    if (!packet || !body || !body_length || length < 2U) { return 0; }
    for (i = 1U; i < length && i <= 4U; ++i) {
        remaining += (size_t)(packet[i] & 127U) * multiplier;
        if (!(packet[i] & 128U)) {
            if (remaining != length - i - 1U) { return 0; }
            *body = packet + i + 1U;
            *body_length = remaining;
            return 1;
        }
        multiplier *= 128U;
    }
    return 0;
}

/**
 * @brief 提取 PUBLISH 的主题、载荷和标志；返回的指针引用原报文，调用方须保证其有效期。
 */
int mqtt_read_publish(const uint8_t *packet, size_t length, mqtt_publish_t *message)
{
    const uint8_t *body;
    size_t body_length;
    size_t offset;
    if (!message || !mqtt_packet_body(packet, length, &body, &body_length) || (packet[0] >> 4) != 3U || body_length < 2U) { return 0; }
    memset(message, 0, sizeof(*message));
    message->qos = (packet[0] >> 1) & 3U;
    message->retained = packet[0] & 1U;
    message->duplicate = (packet[0] >> 3) & 1U;
    if (message->qos > 1U || (!message->qos && message->duplicate)) { return 0; }
    message->topic_length = ((size_t)body[0] << 8) | body[1];
    if (!message->topic_length || message->topic_length > body_length - 2U) { return 0; }
    message->topic = body + 2U;
    offset = message->topic_length + 2U;
    if (message->qos) {
        if (body_length - offset < 2U) { return 0; }
        message->packet_id = (uint16_t)(((uint16_t)body[offset] << 8) | body[offset + 1U]);
        if (!message->packet_id) { return 0; }
        offset += 2U;
    }
    message->payload = body + offset;
    message->payload_length = body_length - offset;
    return 1;
}
