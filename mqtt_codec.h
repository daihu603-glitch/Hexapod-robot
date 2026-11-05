/**
 * @file mqtt_codec.h
 * @brief MQTT 报文编解码：构造网络字节序数据，按 Remaining Length 从 TCP 字节流恢复报文。
 */
#ifndef MQTT_CODEC_H
#define MQTT_CODEC_H
#include <stdint.h>
#include <stddef.h>
#define MQTT_PACKET_CAPACITY 512U

typedef struct {
    uint8_t bytes[MQTT_PACKET_CAPACITY];
    size_t length;
    size_t expected;
} mqtt_stream_t;

typedef struct {
    const uint8_t *topic;
    size_t topic_length;
    const uint8_t *payload;
    size_t payload_length;
    uint16_t packet_id;
    uint8_t qos;
    uint8_t retained;
    uint8_t duplicate;
} mqtt_publish_t;

/**
 * @brief 构造 MQTT 3.1.1 CONNECT；包含清理会话和 offline 遗嘱，按非空参数附加认证字段。
 * 这是通用构包接口，当前本地服务端只接受无认证、无遗嘱的连接配置。
 */
size_t mqtt_connect(uint8_t *out, size_t capacity, const char *client_id,
                    const char *username, const char *password,
                    const char *will_topic, uint16_t keepalive);
/**
 * @brief 构造 SUBSCRIBE，报文标识符 id 必须非零，申请的订阅服务质量为 QoS0。
 */
size_t mqtt_subscribe(uint8_t *out, size_t capacity, const char *topic, uint16_t id);
/**
 * @brief 构造 QoS0 PUBLISH；retained 非零时设置保留位，成功返回长度，失败返回 0。
 */
size_t mqtt_publish(uint8_t *out, size_t capacity, const char *topic, const char *payload, int retained);
/* 逐字节组包返回：1 完整，0 未收齐，-1 非法或超长；消费完整报文后需重置。 */
/**
 * @brief 清空流式组包长度；完整报文消费后必须调用，才能接收下一包。
 */
void mqtt_stream_reset(mqtt_stream_t *stream);
/**
 * @brief 追加一个字节并解析变长长度：1 为完整报文，0 为未收齐，-1 为非法或超长。
 */
int mqtt_stream_feed(mqtt_stream_t *stream, uint8_t byte);
/**
 * @brief 校验长度字段与实际报文一致，返回报文体指针和字节数。
 */
int mqtt_packet_body(const uint8_t *packet, size_t length, const uint8_t **body, size_t *body_length);
/**
 * @brief 提取 PUBLISH 的主题、载荷和标志；返回的指针引用原报文，调用方须保证其有效期。
 */
int mqtt_read_publish(const uint8_t *packet, size_t length, mqtt_publish_t *message);
#endif
