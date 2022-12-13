#include "can_send_task.h"

void can_send_task(void *p_arg);


void can_send_task(void *p_arg)
{

	EventBits_t  EventValue;

	
	while(1)
	{
		EventValue = osEventFlagsWait(EventCommunicationHandle,CHASSIS_SIGNAL | GIMBAL_SIGNAL | SUPERCAP_SIGNAL|SHOOT_SIGNAL,osFlagsWaitAny,osWaitForever);
		
		//云台、底盘、弹仓、发射、超级电容CAN数据等待发送
		if(EventValue & GIMBAL_SIGNAL)
		{
			#if !YAW_SEND_OFF
			CAN1_TX_Gimbal_Yaw();
			#endif
			#if !PITCH_SEND_OFF
			CAN1_TX_Gimbal_Pitch();
			#endif

		}
		else if(EventValue & CHASSIS_SIGNAL)
		{
			#if !CHASSIS_SEND_OFF
			CAN1_TX_Chassis();
			#endif

		}
		else if(EventValue & SUPERCAP_SIGNAL)
		{
			CAN1_TX_SuperCap();
		}
		else if(EventValue & SHOOT_SIGNAL)
		{
			CAN2_TX_Shoot();
		}
	}

}