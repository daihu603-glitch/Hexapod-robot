/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * 【中文说明】CubeMX 生成的 FreeRTOS 装配文件。职责：
  * 1) MX_FREERTOS_Init：在 osKernelInitialize 之后、osKernelStart 之前，
  *    调用 app_rtos_init 创建应用队列/事件对象，创建 defaultTask，
  *    再经 app_rtos_create_threads 补建 servo/distance/log_health 三个任务。
  * 2) StartDefaultTask：defaultTask 入口，直接转入 app_rtos_network_run，
  *    即 1ms 周期的网络/UART4 通信任务（永不返回）。
  * 全部应用代码在 USER CODE 标记内，重新生成代码不会丢失。
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_rtos.h"   /* 应用层 RTOS 装配接口：队列创建、任务创建、network 主循环 */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
/* defaultTask 即应用的 network 通信任务：栈 4KB（承载 MQTT AT 状态机、
 * UART4 协议解析与启动期各模块初始化，是四任务中栈需求最大的），
 * 优先级 Normal——低于 servo/distance 以保证 20ms 控制节拍优先，
 * 高于 log_health 以保证通信吞吐。 */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  *
  * 【中文说明】由 main() 在 osKernelInitialize 之后、osKernelStart 之前调用。
  * 装配顺序很关键：
  * 1) app_rtos_init 先创建队列/事件对象——任务一旦运行就会立即使用它们；
  * 2) 创建 defaultTask（network 任务）；
  * 3) app_rtos_create_threads 登记 defaultTask 句柄并创建 servo/distance/
  *    log_health 三个任务，它们启动后先阻塞在 RTOS_INIT_DONE 事件上，
  *    等 network 任务完成外设初始化后才开始工作。
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  app_rtos_init();   /* 创建 4 个消息队列 + 健康事件标志；失败则 Error_Handler 停机 */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* 本工程任务间全部经队列按值传数据、外设各自独占（I2C 归 servo 任务、
   * USART1 归 log_health、USART3/UART4 归 network 任务），无需互斥量。 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* 无需信号量：启动同步用事件标志 RTOS_INIT_DONE 实现（见 app_rtos.c）。 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* 无需软件定时器：周期节拍由各任务的 osDelayUntil 绝对周期调度实现。 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* 应用队列已在上方 app_rtos_init 中统一创建（log/command/result/distance）。 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  app_rtos_create_threads(defaultTaskHandle);   /* 创建 servo/distance/log_health 三任务并登记句柄 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* 事件标志对象已由 app_rtos_init 创建（health_events）。 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  *
  * 【中文说明】defaultTask 入口：直接转入 app_rtos_network_run（永不返回），
  * 该函数先完成 MQTT/舵机/测距/UART4 的一次性初始化并置 RTOS_INIT_DONE，
  * 然后以 1ms 周期轮询 MQTT 状态机与 ESP32 协议解析，即四任务中的 network 任务。
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;              /* 无任务参数，显式消除未使用告警 */
  app_rtos_network_run();      /* 通信任务主循环，内部 for(;;) 不返回 */
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

