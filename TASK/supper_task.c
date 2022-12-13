#include "supper_task.h"

//下饭.....任务...英语没学好..
void supper_task(void *p_arg);


void supper_task(void *p_arg)
{
	//推荐10Hz
	
	static uint32_t TimeIncrement_supercap;
	TimeIncrement_supercap = osKernelGetTickCount();	
	
	while(1)
	{
		osEventFlagsSet(EventCommunicationHandle,SUPERCAP_SIGNAL);	//超级电容事件组置位
		osDelayUntil(TimeIncrement_supercap + 100);
		TimeIncrement_supercap = osKernelGetTickCount();
	}

}