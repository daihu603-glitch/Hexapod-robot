/**
 * @file board_io.c
 * @brief 板级接口：封装串口中断收发、ESP8266 复位、RGB 灯和蜂鸣器的 HAL 操作。
 *
 * 在系统中的角色：
 * - USART1：调试串口，兼作串口控制台（status/rgb/beep 等命令）与日志输出。
 * - USART3：连接 ESP8266 Wi-Fi 模块，承载 AT 命令与 MQTT over TCP 数据流。
 * - UART4：连接 ESP32-S3 协处理器，是双板通信项目的板间链路。
 * 三路串口均采用"单字节中断接收 + 环形缓冲"模式：中断里只搬运字节，
 * 所有协议解析（AT 状态机、MQTT 组包、板间命令）都在主循环/任务上下文完成，
 * 从而把中断执行时间压到最短，避免丢字节或长时间关中断。
 */
#include "board_io.h"
#include "main.h"
#include "usart.h"
#include "app_rtos.h"
#include <string.h>

/* 串口 HAL 接口依据 FW_F1 V1.8.7 的 stm32f1xx_hal_uart.h。 */
/* Wi-Fi 队列取 4096 字节：+IPD 一次最多可携带约 2048 字节 TCP 数据，
 * 加上 AT 应答文本与多条 MQTT 报文，需要能吸收突发流量而不丢字节。 */
#define WIFI_RING_SIZE 4096U
/* 控制台与 UART4 命令都很短（一行文本），256 字节足够缓冲一行加余量。 */
#define CONSOLE_RING_SIZE 256U
/* 单次发送上限：足够容纳最长的 AT 命令（含 SSID/密码）与一个 MQTT 报文。 */
#define TX_CAPACITY 600U

/**
 * @brief 单生产者-单消费者（SPSC）无锁环形缓冲。
 *
 * 中断安全设计：head 只由串口接收中断（生产者）写入，tail 只由主循环
 * （消费者）写入，双方各自只读对方的索引，因此无需关中断即可安全并发。
 * 队列"满"的判定是 head+1 == tail，即永远保留一个空槽，
 * 这样 head == tail 就唯一地表示"空"，不会与"满"混淆。
 */
typedef struct {
    volatile uint16_t head;   /* 下一个写入位置，仅中断侧修改 */
    volatile uint16_t tail;   /* 下一个读出位置，仅主循环侧修改 */
    uint16_t capacity;        /* 存储区总字节数（含保留空槽） */
    uint8_t *data;            /* 指向静态存储区，不使用动态内存 */
} byte_ring_t;

static uint8_t wifi_storage[WIFI_RING_SIZE];
static uint8_t console_storage[CONSOLE_RING_SIZE];
/* UART4 独立缓冲区，避免 ESP32 命令与 USART1 调试命令互相拼接。
 * 两条链路语义完全不同：USART1 是人工调试控制台，UART4 是 ESP32 板间协议，
 * 若共用缓冲会使两条命令的字节流交错，导致双方解析器都无法切分出完整行。 */
static uint8_t esp_storage[CONSOLE_RING_SIZE];
static byte_ring_t esp_ring = {0U, 0U, CONSOLE_RING_SIZE, esp_storage};
static uint8_t esp_rx_byte;          /* HAL 单字节接收的目标，中断写、主循环读 */
static uint8_t esp_tx[160U];         /* UART4 发送缓冲，容纳一条板间命令 */
static volatile uint8_t esp_error;   /* UART4 溢出/发送失败标志，主循环恢复 */
static byte_ring_t wifi_ring = {0U, 0U, WIFI_RING_SIZE, wifi_storage};
static byte_ring_t console_ring = {0U, 0U, CONSOLE_RING_SIZE, console_storage};
static uint8_t wifi_rx_byte;
static uint8_t console_rx_byte;
static uint8_t wifi_tx[TX_CAPACITY];
static volatile uint8_t tx_busy;     /* Wi-Fi 中断发送进行中标志，发送完成回调清零 */
static volatile uint8_t wifi_error;  /* USART3 溢出/发送失败标志 */
static volatile uint8_t console_error;
static uint8_t rgb_mask;             /* 当前 RGB 位掩码缓存，供状态上报读取 */
static uint8_t beep_active;          /* 蜂鸣器是否处于定时鸣响状态 */
static uint32_t beep_started;        /* 鸣响起始时刻（毫秒时基） */
static uint32_t beep_duration;       /* 本次鸣响持续毫秒数 */

/**
 * @brief 中断侧写入一个字节；队列满返回 0，内存屏障保证先写数据再公布写入位置。
 *
 * __DMB() 的作用：Cortex-M3 上编译器和处理器都可能重排"写数据"与"写 head"
 * 两次存储操作。若消费者先看到 head 前进、后看到数据落盘，就会读到脏字节。
 * 数据内存屏障强制 data[head]=byte 先于 head=next 对其他执行流可见。
 * 满时返回 0（丢字节），由调用方置错误标志触发上层恢复，而不是阻塞中断。
 */
static int ring_push(byte_ring_t *ring, uint8_t byte)
{
    uint16_t next = (uint16_t)((ring->head + 1U) % ring->capacity);
    if (next == ring->tail) { return 0; }
    ring->data[ring->head] = byte;
    __DMB();
    ring->head = next;
    return 1;
}

/**
 * @brief 主循环侧取出一个字节；队列空或输出指针无效返回 0。
 *
 * 对称的 __DMB()：保证先读出 data[tail] 的旧值，再推进 tail 把该槽位
 * 交还给生产者复用，防止乱序导致读到被中断覆写的新数据。
 */
static int ring_pop(byte_ring_t *ring, uint8_t *byte)
{
    if (byte == NULL || ring->head == ring->tail) { return 0; }
    *byte = ring->data[ring->tail];
    __DMB();
    ring->tail = (uint16_t)((ring->tail + 1U) % ring->capacity);
    return 1;
}

/**
 * @brief 关闭灯和蜂鸣器、使能 ESP8266，并开启 USART1 调试串口及 USART3 Wi-Fi 接收中断。
 */
int board_io_init(void)
{
    board_io_rgb(0U);
    board_io_beep(0U);
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    if (HAL_UART_Receive_IT(&huart1, &console_rx_byte, 1U) != HAL_OK) { return 0; }
    if (HAL_UART_Receive_IT(&huart3, &wifi_rx_byte, 1U) != HAL_OK) { return 0; }
    /* UART4 出错只影响 ESP32 控制，不阻断原有 Wi-Fi/手机控制初始化。 */
    if (HAL_UART_Receive_IT(&huart4, &esp_rx_byte, 1U) != HAL_OK) { esp_error = 1U; }
    return 1;
}

/**
 * @brief 返回 HAL 毫秒时基；计时使用无符号差值以兼容计数器回绕。
 */
uint32_t board_io_now(void) { return HAL_GetTick(); }
/**
 * @brief 从 Wi-Fi 接收队列取一个字节；成功返回 1，无数据返回 0。
 */
int board_io_wifi_read(uint8_t *byte) { return ring_pop(&wifi_ring, byte); }
/**
 * @brief 从调试串口接收队列取一个字节；成功返回 1，无数据返回 0。
 */
int board_io_console_read(uint8_t *byte) { return ring_pop(&console_ring, byte); }
/** @brief 通信任务取 UART4 数据；错误恢复时清除缓冲和 ORE，整段输入作废。
 * 错误检查与取字节在同一短临界区，避免消费已报告溢出的残缺数据。
 * HAL 同步中止接收不等待串口数据，不发送数据，不从中断调用。
 *
 * 返回值：-1 表示本次触发了错误恢复（整段旧数据被丢弃，调用方应重置
 * 板间协议解析器）；1 表示成功取出一字节；0 表示无数据且无错误。
 * 之所以在错误后清空整个环形缓冲而非尽力保留：ORE 溢出意味着中间字节
 * 已经丢失，任何残留的半条命令都无法保证完整性，丢弃重来最安全。 */
int board_io_esp_read(uint8_t *byte)
{
    uint32_t mask = __get_PRIMASK();
    int result;
    __disable_irq();
    if (esp_error) {
        (void)HAL_UART_AbortReceive(&huart4);
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        esp_ring.tail = esp_ring.head;
        esp_error = 0U;
        if (HAL_UART_Receive_IT(&huart4, &esp_rx_byte, 1U) != HAL_OK) { esp_error = 1U; }
        result = -1;
    } else {
        result = ring_pop(&esp_ring, byte);
    }
    __set_PRIMASK(mask);
    return result;
}

/**
 * @brief 向 UART4（ESP32 板间链路）中断发送一行文本；成功返回 1。
 *
 * 复制到专用发送缓冲区是因为 HAL_UART_Transmit_IT 期间不能修改源数据，
 * 而调用方文本可能是临时字符串。串口非 READY（上一包未发完）时直接返回 0，
 * 由上层协议自行重发，避免阻塞通信任务。长度上限 160（sizeof(esp_tx)-1）。
 */
int board_io_esp_send(const char *text)
{
    size_t length;
    if (text == NULL || huart4.gState != HAL_UART_STATE_READY) { return 0; }
    length = strlen(text);
    if (length == 0U || length >= sizeof(esp_tx)) { return 0; }
    memcpy(esp_tx, text, length);
    return HAL_UART_Transmit_IT(&huart4, esp_tx, (uint16_t)length) == HAL_OK;
}
/**
 * @brief 查询 Wi-Fi 串口是否仍有中断发送事务未完成。
 */
int board_io_wifi_tx_busy(void) { return tx_busy != 0U; }

/**
 * @brief 在短临界区内取出并清除 Wi-Fi 错误标志，避免与中断同时更新。
 */
int board_io_wifi_fault(void)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t error;
    __disable_irq();
    error = wifi_error;
    wifi_error = 0U;
    __set_PRIMASK(mask);
    return error != 0U;
}

/**
 * @brief 复制数据到专用发送缓冲区并启动中断发送；忙、超长或启动失败返回 0。
 */
int board_io_wifi_send(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0U || length > sizeof(wifi_tx) || tx_busy) { return 0; }
    memcpy(wifi_tx, data, length);
    tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart3, wifi_tx, (uint16_t)length) != HAL_OK) {
        tx_busy = 0U;
        return 0;
    }
    return 1;
}

/**
 * @brief 调度器启动后投递日志队列；启动前直接输出，禁止从中断调用。
 */
void board_io_log(const char *text)
{
    if (text == NULL) { return; }
    if (app_rtos_running()) { app_rtos_log_submit(text); }
    else { board_io_log_write(text); }
}

/** @brief USART1 实际发送接口；运行期仅日志任务调用，其他任务必须走队列。 */
void board_io_log_write(const char *text)
{
    size_t length;
    if (text == NULL) { return; }
    length = strlen(text);
    /* 只允许启动阶段或日志任务使用，限制发送时间，不记录凭据。 */
    if (length > 220U) { length = 220U; }
    if (length && HAL_UART_Transmit(&huart1, (const uint8_t *)text, (uint16_t)length, 30U) != HAL_OK) {
        console_error = 1U;
    }
}

/**
 * @brief asserted 非零时拉低复位脚，并中止收发、清空旧接收数据；为零时释放复位。
 */
void board_io_wifi_reset(int asserted)
{
    uint32_t mask;
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (!asserted) { return; }
    (void)HAL_UART_AbortTransmit(&huart3);
    (void)HAL_UART_AbortReceive(&huart3);
    mask = __get_PRIMASK();
    __disable_irq();
    wifi_ring.head = 0U;
    wifi_ring.tail = 0U;
    wifi_error = 0U;
    tx_busy = 0U;
    __set_PRIMASK(mask);
    if (HAL_UART_Receive_IT(&huart3, &wifi_rx_byte, 1U) != HAL_OK) { wifi_error = 1U; }
}

/**
 * @brief 开始 AT 握手前清除启动噪声、串口溢出标志和旧接收队列，再开启接收。
 */
void board_io_wifi_prepare_rx(void)
{
    uint32_t mask;
    /* 开始 AT 握手前丢弃复位期间的数据与串口错误。 */
    (void)HAL_UART_AbortReceive(&huart3);
    mask = __get_PRIMASK();
    __disable_irq();
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    wifi_ring.head = wifi_ring.tail = 0U;
    wifi_error = 0U;
    __set_PRIMASK(mask);
    if (HAL_UART_Receive_IT(&huart3, &wifi_rx_byte, 1U) != HAL_OK) { wifi_error = 1U; }
}

/**
 * @brief 按位控制 RGB：bit0 红、bit1 绿、bit2 蓝；板载 LED 为低电平点亮。
 */
void board_io_rgb(uint8_t mask)
{
    rgb_mask = mask & 7U;
    HAL_GPIO_WritePin(RED_RGB_GPIO_Port, RED_RGB_Pin, (mask & 1U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_RGB_GPIO_Port, GREEN_RGB_Pin, (mask & 2U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_RGB_GPIO_Port, BLUE_RGB_Pin, (mask & 4U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 返回当前 RGB 位掩码，范围为 0～7。
 */
uint8_t board_io_rgb_get(void) { return rgb_mask; }

/**
 * @brief 开启蜂鸣器指定毫秒，最长 1000 毫秒；传入 0 立即关闭。
 */
void board_io_beep(uint32_t duration_ms)
{
    if (duration_ms > 1000U) { duration_ms = 1000U; }
    beep_duration = duration_ms;
    beep_started = board_io_now();
    beep_active = duration_ms != 0U;
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, beep_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 返回蜂鸣器是否处于定时鸣响状态。
 */
int board_io_beep_active(void) { return beep_active != 0U; }

/**
 * @brief 主循环维护蜂鸣器到时关闭，并尝试恢复调试串口接收。
 */
void board_io_process(void)
{
    if (beep_active && (uint32_t)(board_io_now() - beep_started) >= beep_duration) { board_io_beep(0U); }
    if (console_error) {
        console_error = 0U;
        (void)HAL_UART_AbortReceive(&huart1);
        if (HAL_UART_Receive_IT(&huart1, &console_rx_byte, 1U) != HAL_OK) { console_error = 1U; }
    }
}

/**
 * @brief 接收中断回调：只入队并重新挂接单字节接收，协议解析由主循环完成。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) {
        if (!ring_push(&wifi_ring, wifi_rx_byte)) { wifi_error = 1U; }
        if (HAL_UART_Receive_IT(uart, &wifi_rx_byte, 1U) != HAL_OK) { wifi_error = 1U; }
    } else if (uart == &huart1) {
        if (!ring_push(&console_ring, console_rx_byte)) { console_error = 1U; }
        if (HAL_UART_Receive_IT(uart, &console_rx_byte, 1U) != HAL_OK) { console_error = 1U; }
    } else if (uart == &huart4) {
        /* 只缓存字节和错误，不在中断里解析命令、打印或访问舵机 I2C。 */
        if (!esp_error && !ring_push(&esp_ring, esp_rx_byte)) { esp_error = 1U; }
        if (HAL_UART_Receive_IT(uart, &esp_rx_byte, 1U) != HAL_OK) { esp_error = 1U; }
    }
}

/**
 * @brief 发送完成中断回调：释放 Wi-Fi 发送忙标志。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) { tx_busy = 0U; }
}

/**
 * @brief 串口错误中断回调：仅记录错误，交由主循环恢复。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) { wifi_error = 1U; }
    if (uart == &huart1) { console_error = 1U; }
    if (uart == &huart4) { esp_error = 1U; }
}
