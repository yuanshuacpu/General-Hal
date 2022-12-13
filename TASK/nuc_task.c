#include "nuc_task.h"

/**
 *@Function:		nuc_task(void *p_arg)
 *@Description:	NUC任务
 *@Param:       形参
 *@Return:	  	返回值
 */
 extern int nuc_task_speed;
void nuc_send_task(void *p_arg)
{
	static uint32_t TimeIncrement_nuc;
	TimeIncrement_nuc = osKernelGetTickCount();	
	
	while(1)
	{
		NUC_Send_Data();
		osDelayUntil(TimeIncrement_nuc + nuc_task_speed);
		TimeIncrement_nuc = osKernelGetTickCount();
	}

}