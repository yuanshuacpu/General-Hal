#include "pwm.h"

void ALL_PWM_INIT(void)
{

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // TIM1 1 2 3 4
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // ???
}