/**
 * @file servo_io.h
 * @brief 舵机总线适配：将逻辑总线编号映射到 STM32 HAL I2C，集中处理地址与超时。
 */
#ifndef SERVO_IO_H
#define SERVO_IO_H
#include <stdint.h>
/* 总线 0 对应 I2C1，总线 1 对应 I2C2；地址参数为未左移的七位地址。
 * 仅主循环调用，使用有超时限制的同步事务；成功返回 1。 */
/**
 * @brief 同步写寄存器，最多四字节、超时五毫秒；输入为七位地址，左移后传给 HAL。
 */
int servo_io_write(uint8_t bus, uint8_t address, uint8_t reg,
                   const uint8_t *data, uint16_t length);
/**
 * @brief 同步读取一个八位寄存器；成功返回 1，参数错误或 HAL 通信失败返回 0。
 */
int servo_io_read(uint8_t bus, uint8_t address, uint8_t reg, uint8_t *value);
/**
 * @brief 提供设备初始化所需的毫秒延时，只在主循环上下文使用。
 */
void servo_io_delay_ms(uint32_t milliseconds);
#endif
