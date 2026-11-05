/**
 * @file app_mqtt.h
 * @brief 应用调度：建立 ESP8266 热点和本地 MQTT 服务，统一处理手机与串口控制命令。
 */
#ifndef APP_MQTT_H
#define APP_MQTT_H
/**
 * @brief 初始化板级接口、舵机应用和网络状态；应在 CubeMX 外设初始化后调用。
 */
void app_mqtt_init(void);
/**
 * @brief 主循环周期调用：推进网络状态机、处理控制命令，并按需或每两秒发布状态。
 */
void app_mqtt_process(void);
#include <stddef.h>
/** @brief Network-task-only UART bridge to existing RGB/beep/network functions.
 * Returns 1 for handled commands, 0 for an unrelated command. No gait changes. */
int app_mqtt_link_command(const char *command, char *reply, size_t capacity);
#endif
