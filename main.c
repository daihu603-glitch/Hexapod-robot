/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * 【中文说明】CubeMX 生成的工程入口文件（STM32F103VET6）。
  * 职责：初始化 HAL 与系统时钟（HSE 8MHz × PLL9 = 72MHz SYSCLK），
  * 依次初始化 GPIO、USART1（调试日志）、USART3（ESP8266 Wi-Fi）、
  * UART4（ESP32-S3 双板通信）、I2C1/I2C2（两块 PCA9685 舵机驱动），
  * 然后创建 FreeRTOS 内核对象与任务并启动调度器，main 不再返回。
  * 注意：应用层队列/任务的创建全部在 MX_FREERTOS_Init（freertos.c）中
  * 完成，本文件只做 CubeMX 生成的底层外设初始化。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  *
  * 【中文说明】程序入口：完成 MCU 与外设初始化后启动 FreeRTOS 调度器。
  * 正常情况下 osKernelStart() 之后控制权交给任务，本函数不再返回。
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 外设初始化顺序说明：GPIO 先于串口/I2C（引脚复用需先就绪）；
   * USART1 = 调试日志串口，USART3 = ESP8266 Wi-Fi（AT 指令，MQTT 通道），
   * UART4 = ESP32-S3 双板通信（PC10=TX/PC11=RX，115200 8N1），
   * I2C1/I2C2 = 两块 PCA9685 舵机驱动板（各自总线，同地址 0x40）。 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  /* 应用初始化迁入 MX_FREERTOS_Init，在内核初始化后创建队列和信号量。 */
  /* USER CODE END 2 */

  /* Init scheduler */
  /* 先 osKernelInitialize（初始化 CMSIS-RTOS2 层对象），
   * 再由 MX_FREERTOS_Init 创建应用队列（app_rtos_init）与四个任务。
   * 之所以不在 main 外设初始化前创建，是因为内核对象依赖 FreeRTOS 堆已可用。 */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 正常不会返回此处；调度器启动失败时停止运行，避免退回裸机模式。 */
    Error_Handler();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * 【中文说明】时钟树配置：外部晶振 HSE 8MHz，经 PLL 九倍频得到 72MHz SYSCLK
  * （STM32F103 的最高主频）。AHB 不分频 = 72MHz；APB1 二分频 = 36MHz
  * （APB1 外设上限 36MHz，USART3/UART4/I2C 挂在此总线）；APB2 不分频 = 72MHz
  * （USART1 挂在此总线）。Flash 等待周期设为 2，是 72MHz 主频下的强制要求。
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  *
  * 【中文说明】HAL 时基改由 TIM4 提供（而非 SysTick），因为 SysTick 已被
  * FreeRTOS 内核占用作为调度 tick。TIM4 每 1ms 中断一次驱动 HAL_IncTick，
  * 保证 HAL 超时机制与 board_io_now() 返回的毫秒时基在 RTOS 下仍正确推进。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();  /* 推进 HAL 毫秒计数，供 HAL_GetTick/超时与协议行超时判断使用 */
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  *
  * 【中文说明】全局错误处理：关中断后死循环停机。设计意图是让故障"卡死"
  * 可被调试器/看门狗观察到，而不是带着错误状态继续驱动 18 路舵机（安全风险）。
  * app_rtos.c 中的内核对象创建失败、心跳丢失等致命错误也统一走这里。
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
