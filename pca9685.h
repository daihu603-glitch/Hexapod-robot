/**
 * @file pca9685.h
 * @brief PCA9685 设备驱动：配置约 50 Hz PWM，输出舵机脉冲；总线访问由 servo_io 封装。
 */
#ifndef PCA9685_H
#define PCA9685_H
#include <stdint.h>

typedef enum {
    PCA9685_OK = 0, PCA9685_INVALID, PCA9685_IO_ERROR,
    PCA9685_NOT_READY, PCA9685_CONFIG_ERROR
} pca9685_result_t;

typedef struct {
    uint8_t bus;
    uint8_t address;
    uint8_t ready;
} pca9685_t;

/* 固定使用标称 25 MHz 内部时钟和预分频 121，输出频率约 50 Hz。
 * 初始化关闭全部 16 路输出；调用方须在主循环内串行访问。 */
/**
 * @brief 先关闭输出，再休眠配置预分频、输出模式和自增地址；唤醒后回读寄存器确认配置。
 * 使用标称 25 MHz 内部时钟与预分频 121，得到约 50 Hz；主循环内串行调用。
 */
pca9685_result_t pca9685_init(pca9685_t *device);
/**
 * @brief 设置全局 FULL_OFF 关闭全部 16 路输出；通信失败时清除设备就绪标志。
 */
pca9685_result_t pca9685_all_off(pca9685_t *device);
/* pulse_us 为 0 时关闭通道；非零脉宽范围为 1000～2000 微秒。 */
/**
 * @brief 设置指定 CH0～CH15 的脉宽：0 关闭通道，非零只接受 1000～2000 微秒。
 * 写入成功只表示总线操作完成，实际波形和舵机运动仍需硬件验证。
 */
pca9685_result_t pca9685_set_pulse(pca9685_t *device, uint8_t channel, uint16_t pulse_us);
#endif
