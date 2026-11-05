/**
 * @file mqtt_endpoint.h
 * @brief 本地 MQTT 服务端：维护有限个客户端和订阅，执行控制命令并转发匹配主题。
 */
#ifndef MQTT_ENDPOINT_H
#define MQTT_ENDPOINT_H
#include <stdint.h>
#include <stddef.h>
#define MQTT_CLIENTS 5U

typedef int (*mqtt_send_fn)(uint8_t link, const uint8_t *data, size_t length);
typedef void (*mqtt_close_fn)(uint8_t link);
typedef void (*mqtt_command_fn)(const char *topic, const char *payload);

/* 本地有界 MQTT 3.1.1 服务：清理会话、QoS0、不支持遗嘱和保留消息存储。 */
/**
 * @brief 清空客户端和命令计数，注册发送、关闭连接及业务命令回调。
 */
void mqtt_endpoint_init(mqtt_send_fn send, mqtt_close_fn close, mqtt_command_fn command);
/**
 * @brief TCP 连接建立时清空对应槽位，并开始等待客户端 CONNECT 报文。
 */
void mqtt_endpoint_open(uint8_t link, uint32_t now);
/**
 * @brief TCP 连接断开后释放该槽位的会话、订阅和组包状态。
 */
void mqtt_endpoint_closed(uint8_t link);
/**
 * @brief 按连接逐字节组包；收齐后处理并重置组包器，非法报文触发断开。
 */
void mqtt_endpoint_feed(uint8_t link, uint8_t byte, uint32_t now);
/**
 * @brief 检查超时：CONNECT 等待 10 秒、分包等待 5 秒、心跳等待 Keep Alive 的 1.5 倍。
 */
void mqtt_endpoint_process(uint32_t now);
/**
 * @brief 生成无保留的 QoS0 消息，向所有匹配订阅的已连接客户端各发送一次。
 */
void mqtt_endpoint_publish(const char *topic, const char *payload);
/**
 * @brief 统计完成 MQTT CONNECT 握手的客户端数量。
 */
unsigned mqtt_endpoint_clients(void);
/**
 * @brief 返回本次服务初始化以来通过校验的发布消息数量。
 */
uint32_t mqtt_endpoint_command_count(void);
#endif
