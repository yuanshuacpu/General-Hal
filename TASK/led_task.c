#include "led_task.h"
void led_task(void);

void led_task(void)
{
	int led_frequency = 0;
	while(1)
	{
				//ledµÆÆµÂÊ¿ØÖÆ
		led_frequency++;
		
		if(led_frequency % 1 == 0)
		{
			HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
		}
		
		//·äÃùÆ÷ÏìÉù
		if(led_frequency == 1)
		{
			Beep_Ctrl(1000,10);			
		}
		else if(led_frequency == 2)
		{
			Beep_Ctrl(800,10);						
		}
		else if(led_frequency == 3)
		{
			Beep_Ctrl(600,10);						
		}
		else if(led_frequency == 4)
		{
			Beep_Ctrl(400,10);						
		}
		else
		{
			Beep_Ctrl(10000,0);									
		}
		osDelay(500);
	}
}