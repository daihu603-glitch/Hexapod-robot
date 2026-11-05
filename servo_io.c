/**
 * @file servo_io.c
 * @brief 舵机总线适配：将逻辑总线编号映射到 STM32 HAL I2C，集中处理地址与超时。
 *
 * 在系统中的角色：pca9685 驱动与 HAL 之间的薄适配层。两片 PCA9685 分别
 * 挂在 I2C1（逻辑总线 0）与 I2C2（逻辑总线 1）上，驱动只用逻辑编号，
 * 换板/换总线时只需改这里的映射。同时集中约定：
 * - 七位地址统一在调用处左移 1 位转为 HAL 需要的八位格式；
 * - 所有事务使用 5ms 同步超时：舵机控制走独立任务，超时必须足够短，
 *   避免总线被拉死（如器件不响应 SCL）时阻塞整个任务调度。
 */
#include "servo_io.h"
#include "i2c.h"
#include <string.h>
#include <stddef.h>

#define SERVO_IO_TIMEOUT_MS 5U   /* 单次 I2C 事务超时（毫秒），防止总线卡死阻塞任务 */
#define SERVO_IO_MAX_WRITE 4U    /* 单次写上限：PCA9685 每通道 ON/OFF 共 4 字节 */

/**
 * @brief 逻辑总线 0 对应 hi2c1，逻辑总线 1 对应 hi2c2；其他编号返回空指针。
 */
static I2C_HandleTypeDef *get_bus(uint8_t bus)
{
    if (bus == 0U) { return &hi2c1; }
    if (bus == 1U) { return &hi2c2; }
    return NULL;
}

/**
 * @brief 同步写寄存器，最多四字节、超时五毫秒；输入为七位地址，左移后传给 HAL。
 *
 * 先复制到本地 buffer 再发送：HAL_I2C_Mem_Write 的参数不是 const，
 * 而 pca9685 驱动传入的是常量数组，复制一次避免丢弃 const 限定。
 * address > 0x7F 拒绝，因为七位地址左移后必须能放进 HAL 的八位地址字段。
 * 返回 1 表示 HAL_OK，0 表示参数非法、总线号非法或 I2C 事务失败
 * （NACK/超时/总线错误），由 pca9685 层决定降级动作（如清 ready 标志）。
 */
int servo_io_write(uint8_t bus, uint8_t address, uint8_t reg,
                   const uint8_t *data, uint16_t length)
{
    uint8_t buffer[SERVO_IO_MAX_WRITE];
    I2C_HandleTypeDef *handle = get_bus(bus);
    if (!handle || !data || !length || length > sizeof(buffer) || address > 0x7FU) { return 0; }
    memcpy(buffer, data, length);
    return HAL_I2C_Mem_Write(handle, (uint16_t)(address << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buffer, length, SERVO_IO_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 同步读取一个八位寄存器；成功返回 1，参数错误或 HAL 通信失败返回 0。
 */
int servo_io_read(uint8_t bus, uint8_t address, uint8_t reg, uint8_t *value)
{
    I2C_HandleTypeDef *handle = get_bus(bus);
    if (!handle || !value || address > 0x7FU) { return 0; }
    return HAL_I2C_Mem_Read(handle, (uint16_t)(address << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, value, 1U, SERVO_IO_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 提供设备初始化所需的毫秒延时，只在主循环上下文使用。
 */
void servo_io_delay_ms(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}
