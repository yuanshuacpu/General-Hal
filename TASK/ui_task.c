#include "ui_task.h"


/**
 *@Function:		ui_task(void *p_arg)
 *@Description:	UI����
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void ui_task(void *p_arg)
{	

	while(1)
	{
		//����UI
		Show_UI();
		//����30Hz
		osDelay(20);
	}
}
