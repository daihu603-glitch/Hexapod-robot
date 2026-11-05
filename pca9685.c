/**
 * @file pca9685.c
 * @brief PCA9685 设备驱动：配置约 50 Hz PWM，输出舵机脉冲；总线访问由 servo_io 封装。
 *
 * 在系统中的角色：六足机器人 18 路舵机由 PCA9685 16 通道 PWM 芯片驱动
 * （两片级联，逻辑总线编号区分）。本文件只含与芯片手册相关的寄存器级逻辑：
 * 初始化（时钟/模式/预分频）、单通道脉宽设置、全通道关闭。所有 I2C 读写、
 * 地址换算与超时由 servo_io 提供，保持驱动可移植。
 *
 * 关键换算（依据 PCA9685 数据手册）：
 * - PWM 频率 = 25 MHz / [4096 × (PRE_SCALE + 1)]；
 *   50 Hz 舵机标准帧周期 20ms → PRE_SCALE = round(25e6/(4096×50)) - 1 = 121；
 * - 每计数时长 = (PRE_SCALE+1)/25 MHz = 122/25 ≈ 4.88 µs；
 *   脉宽转计数：ticks = pulse_us × 25 / 122（四舍五入）；
 * - 每通道 4 个寄存器：ON_L/ON_H/OFF_L/OFF_H，各 12 位有效；
 *   OFF_H 的 bit4（FULL_OFF=0x10）强制输出恒低，用于关闭通道。
 */
#include "pca9685.h"
#include "pca9685_reg.h"
#include "servo_io.h"

#define PCA9685_PRESCALE_50HZ 121U  /* 25MHz/(4096*50Hz)-1 = 121.09，取整 121 → 约 50.08 Hz */
#define PCA9685_CHANNELS 16U        /* 单片 PCA9685 的 PWM 通道数 CH0..CH15 */

/**
 * @brief 检查设备对象、总线编号与支持的七位 I2C 地址范围。
 * 地址 0x40..0x5F 为 PCA9685 有效范围（A0..A5 硬件跳线）；此处放宽到
 * 0x77 以兼容同总线上其他地址编址的器件定义，越界一律拒绝，防止误写他设备。
 */
static int valid(const pca9685_t *device)
{
    return device && device->bus < 2U && device->address >= 0x40U && device->address <= 0x77U;
}

/**
 * @brief 向设备的一个寄存器写入一个字节，成功返回 1。
 */
static int write_byte(pca9685_t *device, uint8_t reg, uint8_t value)
{
    return servo_io_write(device->bus, device->address, reg, &value, 1U);
}

/**
 * @brief 设置全局 FULL_OFF 关闭全部 16 路输出；通信失败时清除设备就绪标志。
 */
pca9685_result_t pca9685_all_off(pca9685_t *device)
{
    if (!valid(device)) { return PCA9685_INVALID; }
    /* 单字节写入无需预先开启寄存器地址自增。 */
    if (!write_byte(device, PCA9685_ALL_LED_OFF_H, PCA9685_FULL_OFF)) {
        device->ready = 0U;
        return PCA9685_IO_ERROR;
    }
    return PCA9685_OK;
}

/**
 * @brief 先关闭输出，再休眠配置预分频、输出模式和自增地址；唤醒后回读寄存器确认配置。
 * 使用标称 25 MHz 内部时钟与预分频 121，得到约 50 Hz；主循环内串行调用。
 *
 * 初始化序列（顺序依据数据手册 7.3.1.1）：
 * 1) ALL_LED_OFF_H 写 FULL_OFF：配置期间所有输出保持低电平，舵机不会抖动；
 * 2) 读 MODE1 检查 EXTCLK 位：外部时钟模式只能由掉电/软件复位清除，若已
 *    置位则内部 25 MHz 换算不成立，拒绝继续并报 CONFIG_ERROR；
 * 3) MODE1 写 SLEEP|AI：预分频寄存器只在休眠态可写，同时开启地址自增
 *    （后续可用一次 I2C 事务连写 4 个通道寄存器）；
 * 4) ALL_LED_ON_L 起连写 4 字节，把全局默认 ON/OFF 设为"恒关"；
 * 5) MODE2 写 OUTDRV：推挽输出（而非开漏），直接驱动舵机信号线；
 * 6) 写 PRE_SCALE=121 设定约 50 Hz；
 * 7) MODE1 清 SLEEP（保留 AI）唤醒芯片，等 1ms（手册要求振荡器稳定
 *    ≥500µs）后回读 MODE1/PRE_SCALE 校验生效，全部符合才置 ready。
 * 任何一步 I2C 失败返回 IO_ERROR；回读不符返回 CONFIG_ERROR。
 */
pca9685_result_t pca9685_init(pca9685_t *device)
{
    uint8_t mode;
    uint8_t prescale;
    const uint8_t off[4] = {0U, 0U, 0U, PCA9685_FULL_OFF};
    if (!valid(device)) { return PCA9685_INVALID; }
    device->ready = 0U;
    if (pca9685_all_off(device) != PCA9685_OK) { return PCA9685_IO_ERROR; }
    if (!servo_io_read(device->bus, device->address, PCA9685_MODE1, &mode)) { return PCA9685_IO_ERROR; }
    /* 外部时钟选择位保持到掉电或软件复位；若已置位，不能按内部 25 MHz 时钟配置。 */
    if (mode & PCA9685_EXTCLK) { return PCA9685_CONFIG_ERROR; }
    if (!write_byte(device, PCA9685_MODE1, PCA9685_SLEEP | PCA9685_AUTO_INCREMENT) ||
        !servo_io_write(device->bus, device->address, PCA9685_ALL_LED_ON_L, off, sizeof(off)) ||
        !write_byte(device, PCA9685_MODE2, PCA9685_OUTDRV) ||
        !write_byte(device, PCA9685_PRE_SCALE, PCA9685_PRESCALE_50HZ) ||
        !write_byte(device, PCA9685_MODE1, PCA9685_AUTO_INCREMENT)) { return PCA9685_IO_ERROR; }
    /* 振荡器启动至少需 500 微秒；此处等待 1 毫秒，期间所有通道仍关闭。 */
    servo_io_delay_ms(1U);
    if (!servo_io_read(device->bus, device->address, PCA9685_MODE1, &mode) ||
        !servo_io_read(device->bus, device->address, PCA9685_PRE_SCALE, &prescale)) { return PCA9685_IO_ERROR; }
    if ((mode & (PCA9685_SLEEP | PCA9685_EXTCLK | PCA9685_AUTO_INCREMENT)) != PCA9685_AUTO_INCREMENT ||
        prescale != PCA9685_PRESCALE_50HZ) { return PCA9685_CONFIG_ERROR; }
    device->ready = 1U;
    return PCA9685_OK;
}

/**
 * @brief 设置指定 CH0～CH15 的脉宽：0 关闭通道，非零只接受 1000～2000 微秒。
 * 写入成功只表示总线操作完成，实际波形和舵机运动仍需硬件验证。
 *
 * 参数校验：1000..2000µs 对应标准舵机 0..180° 的脉宽范围，超出会打坏
 * 舵机机械限位，故直接拒绝；pulse_us=0 是合法的"关闭通道"请求。
 * 寄存器布局：通道 n 的 4 字节从 LED0_ON_L + 4n 起（依赖 init 开启的地址
 * 自增），一次 I2C 事务连写 {ON_L, ON_H, OFF_L, OFF_H}：
 * - ON 恒为 0：脉冲从周期起点开始拉高；
 * - OFF：脉宽对应计数 ticks；关闭通道时 OFF_H 置 FULL_OFF(0x10) 恒低。
 * 计数换算：时钟 25 MHz 经 (PRE_SCALE+1)=122 分频后每计数约 4.88µs，
 * ticks = pulse_us × 25 / 122，分子加 61（=122/2）实现四舍五入。
 * 例：1500µs（90° 中位）→ 307 计数；2000µs → 410 计数。
 */
pca9685_result_t pca9685_set_pulse(pca9685_t *device, uint8_t channel, uint16_t pulse_us)
{
    uint16_t ticks;
    uint8_t data[4] = {0U, 0U, 0U, PCA9685_FULL_OFF};
    if (!valid(device) || channel >= PCA9685_CHANNELS ||
        (pulse_us && (pulse_us < 1000U || pulse_us > 2000U))) { return PCA9685_INVALID; }
    if (!device->ready) { return PCA9685_NOT_READY; }
    if (pulse_us) {
        /* 按 25 MHz 时钟、122 分频换算计数并四舍五入：计数 = 微秒数 × 25 / 122。 */
        ticks = (uint16_t)(((uint32_t)pulse_us * 25U + 61U) / 122U);
        data[2] = (uint8_t)ticks;
        data[3] = (uint8_t)(ticks >> 8);
    }
    if (!servo_io_write(device->bus, device->address,
                        (uint8_t)(PCA9685_LED0_ON_L + 4U * channel), data, sizeof(data))) {
        device->ready = 0U;
        return PCA9685_IO_ERROR;
    }
    return PCA9685_OK;
}
