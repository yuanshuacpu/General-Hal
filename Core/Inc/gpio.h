/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define LED_RED_ON PHout(12)=1	//ºìµÆ
#define LED_RED_OFF PHout(12)=0	//ºìµÆ

#define LED_GREEN_ON PHout(11)=1	//ÂÌµÆ	 
#define LED_GREEN_OFF PHout(11)=0	//ÂÌµÆ	 

#define LED_BLUE_ON PHout(10)=1	//À¶µÆ	 
#define LED_BLUE_OFF PHout(10)=0	//À¶µÆ	

#define LED_RED_TOGGLE 		GPIO_ToggleBits(GPIOH,GPIO_Pin_12)	//ºìµÆ
#define LED_GREEN_TOGGLE 	GPIO_ToggleBits(GPIOH,GPIO_Pin_11)	//ÂÌµÆ	 
#define LED_BLUE_TOGGLE 	GPIO_ToggleBits(GPIOH,GPIO_Pin_10)	//À¶µÆ	
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void Init_bmi088_nss();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

