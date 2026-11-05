/**
 * @file pca9685_reg.h
 * @brief PCA9685 寄存器地址与控制位定义。
 */
#ifndef PCA9685_REG_H
#define PCA9685_REG_H
/* PCA9685 寄存器与位定义，依据 NXP Rev.4 第 7.3.1～7.3.5 节。 */
#define PCA9685_MODE1          0x00U
#define PCA9685_MODE2          0x01U
#define PCA9685_LED0_ON_L      0x06U
#define PCA9685_ALL_LED_ON_L   0xFAU
#define PCA9685_ALL_LED_OFF_H  0xFDU
#define PCA9685_PRE_SCALE      0xFEU
#define PCA9685_SLEEP          0x10U
#define PCA9685_AUTO_INCREMENT 0x20U
#define PCA9685_EXTCLK         0x40U
#define PCA9685_OUTDRV         0x04U
#define PCA9685_FULL_OFF       0x10U
#endif
