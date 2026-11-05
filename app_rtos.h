/** @file app_rtos.h
 * @brief CMSIS-RTOS2 任务与消息接口，队列复制数据，不向任务暴露共享缓冲区。
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H
#include "cmsis_os2.h"
#include "hcsr04_io.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    hcsr04_result_t result;
    hcsr04_sample_t sample;
    uint32_t measured_ms;
} app_distance_message_t;

/** @brief 内核初始化后、启动前建立队列并初始化应用；对象创建失败则停止启动。 */
void app_rtos_init(void);
/** @brief 注册 CubeMX 默认任务，并创建舵机、采集、日志监测三个任务。 */
void app_rtos_create_threads(osThreadId_t network_thread);
/** @brief 由 CubeMX 默认任务调用，持续处理网络与串口命令。 */
void app_rtos_network_run(void);
/** @brief 是否已启动调度器；启动前日志直接输出，启动后经队列输出。 */
bool app_rtos_running(void);
/** @brief 将日志复制入有界队列；队列满时丢弃并设置溢出事件，不阻塞生产者。 */
void app_rtos_log_submit(const char *text);
/** @brief 非阻塞提交舵机命令，成功表示已排队，不代表舵机已运动。 */
bool app_rtos_servo_submit(const char *payload);
/** @brief UART4 请求携带非零事务编号，经原舵机队列执行后回传对应结果。 */
bool app_rtos_servo_submit_link(const char *payload, uint32_t request_id);
/** @brief 通信任务取出一条执行结果；out 至少需要 80 字节。 */
bool app_rtos_servo_result(char *out, size_t capacity);
/** @brief 采集任务投递最新测量，单槽邮箱自动淘汰未读取的旧样本。 */
void app_rtos_distance_put(const app_distance_message_t *message);
/** @brief 通信任务非阻塞取出最新测量；无新样本返回 false。 */
bool app_rtos_distance_get(app_distance_message_t *message);
#endif
