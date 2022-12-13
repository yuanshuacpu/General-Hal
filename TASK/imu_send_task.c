#include "imu_send_task.h"

extern int imu_task_speed;
void imu_send_task(void *p_arg)
{
	static uint32_t TimeIncrement_imu;
	TimeIncrement_imu = osKernelGetTickCount();
	
	while(1)
	{
		
		BMI088_Get_Data();//���������ݸ���
		IMU_Ahrs_Update();//��̬����
		IMU_Attitude_Update();//ŷ����ת��
		
		osDelayUntil(TimeIncrement_imu + imu_task_speed);
		TimeIncrement_imu = osKernelGetTickCount();
	}

}