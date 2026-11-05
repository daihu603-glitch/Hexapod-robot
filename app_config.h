/**
 * @file app_config.h
 * @brief 集中配置 Wi-Fi 热点、MQTT 监听端口和控制主题。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ESP8266 创建的本地热点参数；手机与电脑连接该热点后访问本地 MQTT 服务。 */
#define APP_WIFI_SSID       "senyu"
#define APP_WIFI_PASSWORD   "45453968"
#define APP_WIFI_CHANNEL    6U
#define APP_WIFI_IP         "192.168.4.1"
#define APP_MQTT_PORT       1883U
#define APP_TOPIC_RGB       "guide/rgb"
#define APP_TOPIC_BEEP      "guide/beep"
#define APP_TOPIC_STATE     "guide/state"
#define APP_TOPIC_RESULT    "guide/result"
#define APP_TOPIC_SERVO     "guide/servo"
/* 超声波距离：数值单位厘米；测量失败时发布状态文字，避免手机继续显示旧距离。 */
#define APP_TOPIC_DISTANCE  "guide/distance"

#endif
