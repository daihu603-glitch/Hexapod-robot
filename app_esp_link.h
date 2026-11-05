/** @file app_esp_link.h
 * @brief ESP32 双向 UART4 文本控制；事务编号匹配回包，动作通过原舵机队列执行。 */
#ifndef APP_ESP_LINK_H
#define APP_ESP_LINK_H
#include <stdint.h>

/** @brief 在板级串口初始化后调用，只复位解析状态，不自动移动舵机。 */
void app_esp_link_init(void);
/** @brief 通信任务周期调用；有界读取、按行校验、非阻塞提交命令。 */
void app_esp_link_process(void);
/** @brief 仅通信任务调用，将指定事务的最终执行结果排入回包队列。 */
void app_esp_link_result(uint32_t request_id, const char *result);
#endif
