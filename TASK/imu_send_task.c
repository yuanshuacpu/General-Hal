#include "imu_send_task.h"

extern int imu_task_speed;
void imu_send_task(void *p_arg)
{
	static uint32_t TimeIncrement_imu;
	TimeIncrement_imu = osKernelGetTickCount();
	
	while(1)
	{
		
		BMI088_Get_Data();//陀螺仪数据更新
		IMU_Ahrs_Update();//姿态解算
		IMU_Attitude_Update();//欧拉角转化
		
		osDelayUntil(TimeIncrement_imu + imu_task_speed);
		TimeIncrement_imu = osKernelGetTickCount();
	}

}