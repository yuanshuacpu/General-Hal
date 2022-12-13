#include "supper_task.h"

//�·�.....����...Ӣ��ûѧ��..
void supper_task(void *p_arg);


void supper_task(void *p_arg)
{
	//�Ƽ�10Hz
	
	static uint32_t TimeIncrement_supercap;
	TimeIncrement_supercap = osKernelGetTickCount();	
	
	while(1)
	{
		osEventFlagsSet(EventCommunicationHandle,SUPERCAP_SIGNAL);	//���������¼�����λ
		osDelayUntil(TimeIncrement_supercap + 100);
		TimeIncrement_supercap = osKernelGetTickCount();
	}

}