#ifndef __PWM_H
#define __PWM_H
#include "main.h"

#define COVER_ON __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,COVER_CLOSE_VAL)
#define COVER_OFF __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,COVER_OPEN_VAL)

#define COVER_CCR TIM1->CCR1

void ALL_PWM_INIT(void);

#endif