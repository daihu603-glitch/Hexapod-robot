/**
 * @file app_servo.h
 * @brief 舵机命令层：把全局舵机编号和目标角度转换成对应 PCA9685 通道的脉宽。
 */
#ifndef APP_SERVO_H
#define APP_SERVO_H
#include <stddef.h>
/* 用户业务模块，在 MX_I2C1_Init 和 MX_I2C2_Init 之后初始化。
 * 命令示例："0 90"、"9 90"、"0 off"、"off"、"status"。
 * 全局编号 0～8 使用 I2C1，9～17 使用 I2C2；每块通道依次为 0、1、2、4、5、6、8、9、10。 */
/**
 * @brief 启动时尝试关闭两块模块的输出；模块缺失不阻塞 Wi-Fi 启动，也不自动指定角度。
 */
void app_servo_init(void);
/**
 * @brief 处理 <编号> <角度|off>、off 和 status；回复写入 reply，capacity 为缓冲区字节数。
 * 编号 0～8 使用 I2C1，编号 9～17 使用 I2C2；每块依次映射到 CH0、1、2、4、5、6、8、9、10。
 * 角度仅允许 45～135；首次使用某块模块时才配置其 PWM 参数。
 */
void app_servo_command(const char *payload, char *reply, size_t capacity);
/* Complete logical-ID frame; validate all angles before writing any channel. */
int app_servo_frame(const float degrees[18]);
#endif
