#include "ui_task.h"


/**
 *@Function:		ui_task(void *p_arg)
 *@Description:	UI任务
 *@Param:       形参
 *@Return:	  	返回值
 */
void ui_task(void *p_arg)
{	

	while(1)
	{
		//绘制UI
		Show_UI();
		//带宽30Hz
		osDelay(20);
	}
}
