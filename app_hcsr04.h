/** @file app_hcsr04.h
 *  @brief 超声波独立测试应用，通过现有 USART1 调试串口报告测距结果。
 */
#ifndef APP_HCSR04_H
#define APP_HCSR04_H

#include <stdbool.h>
#include <stddef.h>

/** @brief 在板级串口初始化后调用一次，初始化测距计时并报告接线提示。 */
void app_hcsr04_init(void);

/** @brief 仅采集任务每 500ms 调用：执行一次测量，将带时间戳结果投递到队列。 */
void app_hcsr04_process(void);
/** @brief 仅通信任务调用：消费最新测量，更新其私有缓存并投递串口日志。 */
void app_hcsr04_report(void);

/**
 * @brief 格式化最近一次测量，供 MQTT 文本面板显示，不重新触发测量。
 * @param out 输出缓冲区，成功测量为厘米数字，如 50.0；无效测量为状态文字。
 * @param capacity 缓冲区大小，包含字符串结尾；建议至少 32 字节。
 * @return 格式化完整返回 true，空指针或容量不足返回 false。
 */
bool app_hcsr04_format(char *out, size_t capacity);

#endif
