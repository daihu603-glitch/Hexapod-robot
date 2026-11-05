/**
 * @file hcsr04_io.h
 * @brief HC-SR04 板级测量接口：PA2 触发，PB12 浮空输入接收回波。
 */
#ifndef HCSR04_IO_H
#define HCSR04_IO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HCSR04_OK = 0,
    HCSR04_NOT_READY,
    HCSR04_INVALID_ARGUMENT,
    HCSR04_ECHO_HIGH,
    HCSR04_NO_ECHO,
    HCSR04_ECHO_TIMEOUT,
    HCSR04_OUT_OF_RANGE
} hcsr04_result_t;

typedef struct {
    uint32_t pulse_us;     /**< 回波高电平宽度，单位微秒。 */
    uint32_t distance_mm;  /**< 按声速 343m/s 换算的距离，单位毫米。 */
} hcsr04_sample_t;

/**
 * @brief 在时钟、GPIO 和 RTOS 内核初始化后开启 DWT 并建立回波信号量。
 * @return 周期计数器可用返回 true；不可用则禁止开始测量。
 */
bool hcsr04_io_init(void);

/**
 * @brief 采集任务发起测量，PB12 双边沿中断采集脉宽，通过信号量等待结束。
 * @param sample 输出数据；仅返回 HCSR04_OK 时距离有效，超范围时保留原始数据。
 * @return 测量状态；无回波或不结束时在 35ms 后退出，禁止从中断内调用。
 * @note 回波等待让出 CPU；中断响应延迟仍影响精度，两次触发需间隔至少 60ms。
 */
hcsr04_result_t hcsr04_io_measure(hcsr04_sample_t *sample);

#endif
