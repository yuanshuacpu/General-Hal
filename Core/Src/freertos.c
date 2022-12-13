/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis_start */
osThreadId_t chassis_startHandle;
const osThreadAttr_t chassis_start_attributes = {
  .name = "chassis_start",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for can_send_start */
osThreadId_t can_send_startHandle;
const osThreadAttr_t can_send_start_attributes = {
  .name = "can_send_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for gimbal_start */
osThreadId_t gimbal_startHandle;
const osThreadAttr_t gimbal_start_attributes = {
  .name = "gimbal_start",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for imu_start */
osThreadId_t imu_startHandle;
const osThreadAttr_t imu_start_attributes = {
  .name = "imu_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for nuc_start */
osThreadId_t nuc_startHandle;
const osThreadAttr_t nuc_start_attributes = {
  .name = "nuc_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for supper_start */
osThreadId_t supper_startHandle;
const osThreadAttr_t supper_start_attributes = {
  .name = "supper_start",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for led_task_start */
osThreadId_t led_task_startHandle;
const osThreadAttr_t led_task_start_attributes = {
  .name = "led_task_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for shoot_start */
osThreadId_t shoot_startHandle;
const osThreadAttr_t shoot_start_attributes = {
  .name = "shoot_start",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for mode_start */
osThreadId_t mode_startHandle;
const osThreadAttr_t mode_start_attributes = {
  .name = "mode_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ui_start */
osThreadId_t ui_startHandle;
const osThreadAttr_t ui_start_attributes = {
  .name = "ui_start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EventCommunication */
osEventFlagsId_t EventCommunicationHandle;
const osEventFlagsAttr_t EventCommunication_attributes = {
  .name = "EventCommunication"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void chassis_task(void *argument);
void can_send_task(void *argument);
void gimbal_task(void *argument);
void imu_send_task(void *argument);
void nuc_send_task(void *argument);
void supper_task(void *argument);
void led_task(void *argument);
void shoot_task(void *argument);
void mode_task(void *argument);
void ui_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chassis_start */
  chassis_startHandle = osThreadNew(chassis_task, NULL, &chassis_start_attributes);

  /* creation of can_send_start */
  can_send_startHandle = osThreadNew(can_send_task, NULL, &can_send_start_attributes);

  /* creation of gimbal_start */
  gimbal_startHandle = osThreadNew(gimbal_task, NULL, &gimbal_start_attributes);

  /* creation of imu_start */
  imu_startHandle = osThreadNew(imu_send_task, NULL, &imu_start_attributes);

  /* creation of nuc_start */
  nuc_startHandle = osThreadNew(nuc_send_task, NULL, &nuc_start_attributes);

  /* creation of supper_start */
  supper_startHandle = osThreadNew(supper_task, NULL, &supper_start_attributes);

  /* creation of led_task_start */
  led_task_startHandle = osThreadNew(led_task, NULL, &led_task_start_attributes);

  /* creation of shoot_start */
  shoot_startHandle = osThreadNew(shoot_task, NULL, &shoot_start_attributes);

  /* creation of mode_start */
  mode_startHandle = osThreadNew(mode_task, NULL, &mode_start_attributes);

  /* creation of ui_start */
  ui_startHandle = osThreadNew(ui_task, NULL, &ui_start_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of EventCommunication */
  EventCommunicationHandle = osEventFlagsNew(&EventCommunication_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		static int chassis_sus_flag=1;
		static int gimbal_sus_flag=1;
		if(CHASSIS_SUSPEND&&chassis_sus_flag)
		{
			osThreadSuspend(chassis_startHandle);
			chassis_sus_flag = 0;
		}
		else if(GIMBAL_SUSPEND&&gimbal_sus_flag)
		{
			osThreadSuspend(gimbal_startHandle);
			gimbal_sus_flag = 0;
		}
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chassis_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_can_send_task */
/**
* @brief Function implementing the can_send_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_send_task */
__weak void can_send_task(void *argument)
{
  /* USER CODE BEGIN can_send_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can_send_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbal_craet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void *argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_imu_send_task */
/**
* @brief Function implementing the imu_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_send_task */
__weak void imu_send_task(void *argument)
{
  /* USER CODE BEGIN imu_send_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_send_task */
}

/* USER CODE BEGIN Header_nuc_send_task */
/**
* @brief Function implementing the nuc_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_nuc_send_task */
__weak void nuc_send_task(void *argument)
{
  /* USER CODE BEGIN nuc_send_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END nuc_send_task */
}

/* USER CODE BEGIN Header_supper_task */
/**
* @brief Function implementing the supper_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_supper_task */
__weak void supper_task(void *argument)
{
  /* USER CODE BEGIN supper_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END supper_task */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the led_task_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
__weak void led_task(void *argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_shoot_task */
/**
* @brief Function implementing the shoot_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_task */
__weak void shoot_task(void *argument)
{
  /* USER CODE BEGIN shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_mode_task */
/**
* @brief Function implementing the myTask10 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mode_task */
__weak void mode_task(void *argument)
{
  /* USER CODE BEGIN mode_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mode_task */
}

/* USER CODE BEGIN Header_ui_task */
/**
* @brief Function implementing the ui_start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ui_task */
__weak void ui_task(void *argument)
{
  /* USER CODE BEGIN ui_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ui_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

