/**
 * @file app_hcsr04.c
 * @brief 输出超声波测试结果，并缓存测量供 MQTT 读取，不自动控制舵机。
 *
 * 在系统中的角色：桥接底层 hcsr04_io（DWT+EXTI 测距）与上层 MQTT/日志。
 * 采集在 app_rtos 的独立任务里周期调用 app_hcsr04_process 完成，结果经
 * RTOS 队列传给通信任务；通信任务调用 app_hcsr04_report 取队列、更新缓存、
 * 打印日志，再由 app_mqtt 通过 app_hcsr04_format 把最新距离格式化上报手机。
 * 本文件只做"测量-缓存-上报"，不参与任何避障/舵机决策。
 */
#include "app_hcsr04.h"
#include "hcsr04_io.h"
#include "board_io.h"
#include "app_rtos.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define HCSR04_LOG_CAPACITY (128U)
/* 结果保鲜期 1500ms = 采集周期 500ms 的 3 倍；超过则视为 stale，
 * 允许个别采样丢失仍能上报有效值，但连续丢失会及时反映为过期。 */
#define HCSR04_STALE_MS (1500U)

static bool enabled;             /* DWT/EXTI 初始化成功后置真；一次失败即永久停用 */
/* 以下缓存仅通信任务读写；采集任务经队列传值，禁止直接修改此缓存。 */
static hcsr04_result_t last_result = HCSR04_NOT_READY;
static uint32_t last_distance_mm;  /* 最近一次成功测得的距离（毫米） */
static uint32_t last_measured_ms;  /* 该结果的采样时刻，用于 stale 判定 */

/**
 * @brief 初始化底层测距硬件并复位缓存；根据结果打印就绪或失败日志。
 * 失败（DWT 时钟不可用）时 enabled 置假，后续 process 直接返回 NOT_READY。
 */
void app_hcsr04_init(void)
{
    enabled = hcsr04_io_init();
    last_result = HCSR04_NOT_READY;
    last_distance_mm = 0U;
    last_measured_ms = 0U;
    board_io_log(enabled ? "[HCSR04] ready: TRIG=PA2 ECHO=PB12 EXTI, interval=500ms\r\n" :
                           "[HCSR04] init failed: DWT clock unavailable\r\n");
}

/**
 * @brief 采集任务上下文调用：执行一次测距并把结果连同时间戳投递到 RTOS 队列。
 * 一旦返回 NOT_READY（硬件不再可用）即永久关闭 enabled，停止后续触发。
 * 本函数运行在采集任务，只写队列、不碰 last_* 缓存。
 */
void app_hcsr04_process(void)
{
    app_distance_message_t message;
    message.sample.pulse_us = message.sample.distance_mm = 0U;
    message.result = enabled ? hcsr04_io_measure(&message.sample) : HCSR04_NOT_READY;
    message.measured_ms = board_io_now();
    if (message.result == HCSR04_NOT_READY) { enabled = false; }
    app_rtos_distance_put(&message);
}

/**
 * @brief 通信任务上下文调用：从队列取一条测量，更新缓存并打印一行日志。
 * 成功结果按厘米（一位小数）与回波脉宽微秒打印；各类失败映射为可诊断的
 * 文字提示（接线、供电、目标过远、DWT 时钟等），方便现场排障。
 * 只有完整未截断的行才输出，避免打印被 sizeof(text) 截断的残句。
 */
void app_hcsr04_report(void)
{
    app_distance_message_t message;
    hcsr04_sample_t sample;
    hcsr04_result_t result;
    const char *reason;
    char text[HCSR04_LOG_CAPACITY];
    int written;
    if (!app_rtos_distance_get(&message)) { return; }
    sample = message.sample;
    result = message.result;
    last_result = result;
    last_distance_mm = result == HCSR04_OK ? sample.distance_mm : 0U;
    last_measured_ms = message.measured_ms;
    if (result == HCSR04_OK) {
        written = snprintf(text, sizeof(text), "[HCSR04] distance=%lu.%lu cm, pulse=%lu us\r\n",
                           (unsigned long)(sample.distance_mm / 10U),
                           (unsigned long)(sample.distance_mm % 10U),
                           (unsigned long)sample.pulse_us);
    } else {
        switch (result) {
        case HCSR04_ECHO_HIGH: reason = "echo high before trigger: check PB12/wiring"; break;
        case HCSR04_NO_ECHO: reason = "no echo: check 5V/GND/TRIG/ECHO or target"; break;
        case HCSR04_ECHO_TIMEOUT: reason = "echo timeout: target too far or ECHO stuck high"; break;
        case HCSR04_OUT_OF_RANGE: reason = "out of range (2..400cm)"; break;
        default:
            reason = "measurement unavailable: restart to check DWT clock";
            break;
        }
        written = snprintf(text, sizeof(text), "[HCSR04] %s\r\n", reason);
    }
    if (written > 0 && (size_t)written < sizeof(text)) { board_io_log(text); }
}

/**
 * @brief 将最新测距结果格式化为字符串供 MQTT 上报；成功写入返回 true。
 *
 * 输出规则：
 * - 结果已过期（超过 HCSR04_STALE_MS 未更新）统一输出 "stale"，让手机端
 *   知道当前值不可信，而不是继续显示一个陈旧距离；
 * - 有效测距输出厘米一位小数（如 "12.3"），毫米整数除以 10 拆分整数/小数位；
 * - 各类失败映射为固定英文状态字（echo_high/no_echo/timeout/out_of_range/
 *   not_ready），便于手机程序按字符串判定。
 * 使用无符号差值判过期，兼容毫秒计数回绕；缓冲区不足时 snprintf 返回被截断，
 * 通过 (size_t)written < capacity 拒绝，返回 false 表示未产生完整输出。
 */
bool app_hcsr04_format(char *out, size_t capacity)
{
    const char *status;
    int written;
    if (out == NULL || capacity == 0U) { return false; }
    if (last_result != HCSR04_NOT_READY && (uint32_t)(board_io_now() - last_measured_ms) > HCSR04_STALE_MS) {
        written = snprintf(out, capacity, "%s", "stale");
        return written >= 0 && (size_t)written < capacity;
    }
    if (last_result == HCSR04_OK) {
        written = snprintf(out, capacity, "%lu.%lu",
                           (unsigned long)(last_distance_mm / 10U),
                           (unsigned long)(last_distance_mm % 10U));
    } else {
        switch (last_result) {
        case HCSR04_ECHO_HIGH: status = "echo_high"; break;
        case HCSR04_NO_ECHO: status = "no_echo"; break;
        case HCSR04_ECHO_TIMEOUT: status = "timeout"; break;
        case HCSR04_OUT_OF_RANGE: status = "out_of_range"; break;
        default: status = "not_ready"; break;
        }
        written = snprintf(out, capacity, "%s", status);
    }
    return written >= 0 && (size_t)written < capacity;
}
