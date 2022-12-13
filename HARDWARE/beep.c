#include "beep.h"


void Beep_Ctrl(uint16_t tune, uint16_t sound)	
{
	htim4.Instance->ARR = tune;
  htim4.Init.Period = tune;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,sound);
}