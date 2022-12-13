/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

#include "file_reference.h"
/* USER CODE BEGIN Includes */

///*
//#define USART_RX_BUF_LENGHT 1024
//#define NUC_TX_BUFF_SIZE 23
//#define NUC_RX_BUFF_SIZE 20

//事件组标志位
//#define GIMBAL_SIGNAL   ( 1 << 0 )
//#define CHASSIS_SIGNAL ( 1 << 1 )
//#define SHOOT_SIGNAL     ( 1 << 2 )
//#define SUPERCAP_SIGNAL     ( 1 << 3 )
//#define SHOOT_EXE_SIGNAL    (1 << 4)

//LED定义 G11 R12 B10
//#define LED_GREEN_ON HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET)
//#define LED_GREEN_OFF HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET)
//#define LED_BLUE_ON HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET)
//#define LED_BLUE_OFF HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET)
//#define LED_RED_ON HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET)
//#define LED_RED_OFF HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET)

//typedef signed char int8_t;
//typedef signed short int int16_t;
//typedef signed int int32_t;
//typedef signed long long int64_t;

///* exact-width unsigned integer types */
//typedef unsigned char uint8_t;
//typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;
//typedef unsigned long long uint64_t;
//typedef unsigned char bool_t;
//typedef float fp32;
//typedef double fp64;

///* USER CODE END Includes */

///* Exported types ------------------------------------------------------------*/
///* USER CODE BEGIN ET */
//操作系统
//#include "cmsis_os2.h"                  
//#include "FreeRTOS.h"
//#include "task.h"
//#include "event_groups.h"
//模块
//#include "ramp.h"
//#include "can_h.h"
//#include "bmi088.h"
//#include "imu.h"
//#include "BMI088driver.h"
//#include "gpio.h"
//#include "dr16.h"
//#include "pid.h"
//#include "flash.h"
//#include "beep.h"
//#include "pwm.h"
//#include "crc.h"
//#include "referee_info.h"
//#include "referee.h"
//#include "string.h"
//#include "nuc.h"
//#include "gimbal.h"
//#include "custom_ui.h"
//#include "debug.h"
//任务
//#include "chassis_task.h"
//#include "gimbal_task.h"
//#include "shoot_task.h"
//#include "nuc_task.h"
//#include "can_send_task.h"
//#include "led_task.h"
//#include "supper_task.h"
//#include "imu_send_task.h"
//#include "mode_task.h"
//#include "ui_task.h"
///* USER CODE END ET */

///* Exported constants --------------------------------------------------------*/
///* USER CODE BEGIN EC */
//extern int imu_tick_counts;
//extern float Yaw_Angle ;//随动，积分角度
//extern char Robot_Color;
//extern int power_relay ;
//extern int power_flag ;
//标志
//extern int CAN1_Signal ;																		//CAN1信号量标志
//extern int CAN2_Signal ;																		//CAN2信号量标志
//extern int SuperCap_Signal ;																//超级电容信号量标志
//extern int DR16_Signal;
//pitch角度
//extern float Pitch_Aim_Angle ;
//extern float Pitch_Angle_Max ;
//extern float Pitch_Angle_Min ;
//模式定义
//extern Gimbal_Mode_Enum Gimbal_Mode;
//extern Shoot_Mode_Enum Shoot_Mode;
//extern Chassis_Mode_Enum Chassis_Mode;
//extern Chassis_Control_Speed_Typedef rc;
//extern Chassis_Control_Speed_Typedef keyboard;	
//extern Shoot_Cmd_Enum Shoot_Cmd ;
//热量
//extern int heat_control ; //热量控制
//extern int heat_remain ; //剩余热量
//基本
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim4;
//extern TIM_HandleTypeDef htim6;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart6;
//extern DMA_HandleTypeDef hdma_usart3_rx;
//extern DMA_HandleTypeDef hdma_usart6_rx;
//extern DMA_HandleTypeDef hdma_usart6_tx;
//extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;

//extern osEventFlagsId_t EventCommunicationHandle;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
