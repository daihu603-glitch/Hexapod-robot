/**
 * @file board_io.h
 * @brief 板级接口：封装串口中断收发、ESP8266 复位、RGB 灯和蜂鸣器的 HAL 操作。
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H
#include <stdint.h>
#include <stddef.h>

/**
 * @brief 关闭灯和蜂鸣器、使能 ESP8266，并开启 USART1 调试串口及 USART3 Wi-Fi 接收中断。
 */
int board_io_init(void);
/**
 * @brief 主循环维护蜂鸣器到时关闭，并尝试恢复调试串口接收。
 */
void board_io_process(void);
/**
 * @brief 返回 HAL 毫秒时基；计时使用无符号差值以兼容计数器回绕。
 */
uint32_t board_io_now(void);
/**
 * @brief 从 Wi-Fi 接收队列取一个字节；成功返回 1，无数据返回 0。
 */
int board_io_wifi_read(uint8_t *byte);
/**
 * @brief 从调试串口接收队列取一个字节；成功返回 1，无数据返回 0。
 */
int board_io_console_read(uint8_t *byte);
/** @brief 非阻塞读取 UART4：1 有字节，0 无数据，-1 表示错误并已尝试重启接收。
 * 返回 -1 后，调用方必须丢弃残缺行直到下一个换行，防止错误拼接成动作命令。 */
int board_io_esp_read(uint8_t *byte);
/** @brief UART4 复制后中断发送；忙或错误返回0，调用者保留报文稍后再试。 */
int board_io_esp_send(const char *text);
/**
 * @brief 复制数据到专用发送缓冲区并启动中断发送；忙、超长或启动失败返回 0。
 */
int board_io_wifi_send(const uint8_t *data, size_t length);
/**
 * @brief 查询 Wi-Fi 串口是否仍有中断发送事务未完成。
 */
int board_io_wifi_tx_busy(void);
/**
 * @brief 在短临界区内取出并清除 Wi-Fi 错误标志，避免与中断同时更新。
 */
int board_io_wifi_fault(void);
/**
 * @brief asserted 非零时拉低复位脚，并中止收发、清空旧接收数据；为零时释放复位。
 */
void board_io_wifi_reset(int asserted);
/**
 * @brief 开始 AT 握手前清除启动噪声、串口溢出标志和旧接收队列，再开启接收。
 */
void board_io_wifi_prepare_rx(void);
/**
 * @brief 通过调试串口输出文本；仅主循环调用，单次最多 220 字节并限制发送等待时间。
 */
void board_io_log(const char *text);
/** @brief 原始串口日志输出，仅启动阶段或日志任务调用，其他任务使用 board_io_log。 */
void board_io_log_write(const char *text);
/**
 * @brief 按位控制 RGB：bit0 红、bit1 绿、bit2 蓝；板载 LED 为低电平点亮。
 */
void board_io_rgb(uint8_t mask);
/**
 * @brief 返回当前 RGB 位掩码，范围为 0～7。
 */
uint8_t board_io_rgb_get(void);
/**
 * @brief 开启蜂鸣器指定毫秒，最长 1000 毫秒；传入 0 立即关闭。
 */
void board_io_beep(uint32_t duration_ms);
/**
 * @brief 返回蜂鸣器是否处于定时鸣响状态。
 */
int board_io_beep_active(void);
#endif
