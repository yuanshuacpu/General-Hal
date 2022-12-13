#ifndef FILE_REFERENCE_H
#define FILE_REFERENCE_H
// #include "main.h"

#define USART_RX_BUF_LENGHT 1024
#define NUC_TX_BUFF_SIZE 23
#define NUC_RX_BUFF_SIZE 20

// �¼����־λ
#define GIMBAL_SIGNAL (1 << 0)
#define CHASSIS_SIGNAL (1 << 1)
#define SHOOT_SIGNAL (1 << 2)
#define SUPERCAP_SIGNAL (1 << 3)
#define SHOOT_EXE_SIGNAL (1 << 4)

// LED���� G11 R12 B10
#define LED_GREEN_ON HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET)
#define LED_GREEN_OFF HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_BLUE_ON HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET)
#define LED_BLUE_OFF HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET)
#define LED_RED_ON HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_RED_OFF HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET)

// hal���ļ�
#include "stm32f4xx_hal.h"

// ����ϵͳ
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
// ģ��
#include "ramp.h"
#include "can_h.h"
#include "bmi088.h"
#include "imu.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "dr16.h"
#include "pid.h"
#include "flash.h"
#include "beep.h"
#include "pwm.h"
#include "crc.h"
#include "referee_info.h"
#include "referee.h"
#include "string.h"
#include "nuc.h"
#include "gimbal.h"
#include "custom_ui.h"
#include "debug.h"
// ����
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "nuc_task.h"
#include "can_send_task.h"
#include "led_task.h"
#include "supper_task.h"
#include "imu_send_task.h"
#include "mode_task.h"
#include "ui_task.h"

// �����ܺ�
#include "type_module.h"
// ���Ͷ���
#include "type_define.h"
#include "mytype.h"

#endif