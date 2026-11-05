/**
 * @file esp_at.h
 * @brief ESP8266 AT 传输层：识别文本应答、+IPD 数据和连接事件，以状态机串行发送。
 */
#ifndef ESP_AT_H
#define ESP_AT_H
#include <stdint.h>
#include <stddef.h>

typedef void (*esp_data_fn)(uint8_t link, uint8_t byte);
typedef void (*esp_link_fn)(uint8_t link, int opened);
/**
 * @brief 注册数据与连接回调，清空 AT 状态和上次事务结果。
 */
void esp_at_init(esp_data_fn data_callback, esp_link_fn link_callback);
/**
 * @brief 主循环处理接收队列和发送阶段，每次限制处理预算，并检测数据与命令超时。
 */
void esp_at_process(void);
/**
 * @brief 追加回车换行并启动一条 AT 命令；timeout_ms 指定整个事务的超时时间。
 */
int esp_at_command(const char *command, uint32_t timeout_ms);
/**
 * @brief 复制网络载荷并发送 CIPSEND；等待提示符后由处理函数发送实际数据。
 */
int esp_at_send(uint8_t link, const uint8_t *data, size_t length);
/**
 * @brief 为编号 0～4 的 TCP 连接发起关闭命令。
 */
int esp_at_close(uint8_t link);
/* 读取结果会清除完成状态：0 等待或空闲，1 成功，-1 错误，-2 超时或故障。 */
/**
 * @brief 取出并清除事务结果：0 等待或空闲，1 成功，-1 命令错误，-2 超时或故障。
 */
int esp_at_result(void);
/**
 * @brief 命令执行中、结果尚未读取或串口仍在发送时返回非零。
 */
int esp_at_busy(void);
/**
 * @brief 查询传输故障；出现故障后由上层复位网络并重新初始化。
 */
int esp_at_fault(void);
#endif
