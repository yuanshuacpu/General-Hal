#include "pid.h"

/*
 *@title：PID数据赋值
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
void Pid_Set(
	PID_TypeDef *PID, float Kp, float Ki, float Kd,
	float Pmax, float Imax, float Dmax, float PIDmax,
	float Speed_Ratio, float Acceletation,
	float Error_Ignored, u8 Motor_Type)
{
	PID->P = Kp;
	PID->I = Ki;
	PID->D = Kd;

	PID->PMax = Pmax;
	PID->IMax = Imax;
	PID->DMax = Dmax;
	PID->PIDOutMax = PIDmax;

	PID->ErrorIgnored = Error_Ignored;

	PID->Speed_Ratio = Speed_Ratio;
	PID->Acceleration = Acceletation;

	PID->Motor_Type = Motor_Type;
}

/*
 *@title：PID数据初始化
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
void Pid_Reset(PID_TypeDef *PID)
{
	PID->P = 0;
	PID->I = 0;
	PID->D = 0;
	PID->CurrentError = 0;
	PID->LastError = 0;
	PID->ErrorIgnored = 0;
	PID->Pout = 0;
	PID->Iout = 0;
	PID->Dout = 0;
	PID->PIDout = 0;
	PID->PIDOutCompensate = 0;
	PID->PMax = 0;
	PID->IMax = 0;
	PID->DMax = 0;
	PID->PIDOutMax = 0;
	PID->PIDOutLast = 0;
	PID->Target_Speed_Last = 0;
	PID->Speed_Ratio = 1;
	PID->Acceleration = 0;
}
/*
 *@title：PID计算函数
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
float Pid_Calc(PID_TypeDef *PID, float Current_Speed, float Target_Speed)
{
	Current_Speed = Current_Speed / PID->Speed_Ratio;

	PID->Target_Speed_Last = Target_Speed;

	// 死区忽略
	PID->CurrentError = Target_Speed - Current_Speed;
	if (fabs(PID->CurrentError) < PID->ErrorIgnored)
		PID->CurrentError = 0;

	// Pout
	PID->Pout = PID->P * PID->CurrentError;
	if (PID->Pout > PID->PMax)
		PID->Pout = PID->PMax;
	if (PID->Pout < -PID->PMax)
		PID->Pout = -PID->PMax;

	// Iout
	PID->Iout += PID->I * PID->CurrentError;
	if (PID->Iout > PID->IMax)
		PID->Iout = PID->IMax;
	if (PID->Iout < -PID->IMax)
		PID->Iout = -PID->IMax;

	// Dout
	PID->Dout = PID->D * (PID->CurrentError - PID->LastError);
	if (PID->Dout > PID->DMax)
		PID->Dout = PID->DMax;
	if (PID->Dout < -PID->DMax)
		PID->Dout = -PID->DMax;

	// PIDout
	PID->PIDout = PID->Pout + PID->Iout + PID->Dout + PID->PIDOutCompensate;

	if (PID->PIDout > PID->PIDOutMax)
		PID->PIDout = PID->PIDOutMax;
	if (PID->PIDout < -PID->PIDOutMax)
		PID->PIDout = -PID->PIDOutMax;

	PID->PIDOutLast = PID->PIDout;
	PID->LastError = PID->CurrentError;

	return PID->PIDout;
}
void Chassis_PID_Init()
{
	for (int i = 0; i < 4; i++)
	{
		Pid_Reset(&PID_Chassis[i]);
		// 注意减速比
		Pid_Set(&PID_Chassis[i], 8.0f, 0.1f, 1.0f, 16000, 500, 5000, 16000, 1, 5000, 0, 0);
	}

	Pid_Reset(&PID_Chassis_Omega);

	Pid_Set(&PID_Chassis_Omega, 100.0f, 0, 100.0f, 16000, 50, 16000, 7000, 1, 16000, 0, 2);

	Pid_Reset(&PID_Storage_Angle);
	Pid_Reset(&PID_Storage_Speed);

	Pid_Set(&PID_Storage_Angle, 1.0f, 0, 0.0f, 10000, 50, 10000, 10000, 1, 10000, 0, 2);

	Pid_Set(&PID_Storage_Speed, 5.0, 0, 0.0f, 10000, 50, 10000, 10000, 1, 10000, 0, 2);
}
void Gimbal_PID_Init()
{
	Pid_Reset(&PID_Gimbal_Angle[0]);
	Pid_Reset(&PID_Gimbal_Angle[1]);
	Pid_Reset(&PID_Gimbal_Speed[0]);
	Pid_Reset(&PID_Gimbal_Speed[1]);

	Pid_Reset(&PID_Trigger[0]);

	Pid_Set(&PID_Trigger[0], 10, 0, 0, 10000, 50, 10000, 10000, 1, 10000, 0, 2);
	Pid_Set(&PID_Gimbal_Angle[1], 2, 0, 0, 10000, 0, 30000, 10000, 1, 30000, 0, 2);		 // Pitch
	Pid_Set(&PID_Gimbal_Speed[1], 60, 0, 0, 30000, 10000, 30000, 30000, 1, 30000, 0, 2); // Pitch

	Pid_Set(&PID_Gimbal_Angle[0], 1.5, 0, 0, 10000, 3000, 30000, 10000, 1, 30000, 0, 2); // Yaw
	Pid_Set(&PID_Gimbal_Speed[0], 150, 1, 0, 30000, 5000, 200, 30000, 1, 30000, 0, 2);	 // Yaw

	Pid_Set(&PID_Gimbal_Angle_Stop[0], 60, 0, 0, 10000, 2000, 30000, 10000, 1, 30000, 0, 2); // Yaw轴电机急停
	Pid_Set(&PID_Gimbal_Speed_Stop[0], 250, 0, 0, 30000, 2000, 200, 30000, 1, 30000, 0, 2);	 // Yaw180

	Pid_Set(&PID_Gimbal_Angle_Stop[1], 15, 0, 0, 10000, 2000, 30000, 10000, 1, 30000, 0, 2); // Pitch轴电机急停
	Pid_Set(&PID_Gimbal_Speed_Stop[1], 30, 0, 0, 30000, 2000, 200, 30000, 1, 30000, 0, 2);	 // Pitch180
}

void Shoot_PID_Init()
{
	Pid_Reset(&PID_Shoot[0]);
	Pid_Reset(&PID_Shoot[1]);
	Pid_Reset(&PID_Shoot[0]);
	Pid_Reset(&PID_Shoot[1]);
	Pid_Reset(&PID_Trigger_Angle);
	Pid_Reset(&PID_Trigger_Speed);

	Pid_Set(&PID_Shoot[0], 10, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
	Pid_Set(&PID_Shoot[1], 10, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
	Pid_Set(&PID_Trigger_Angle, 0.2, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
	Pid_Set(&PID_Trigger_Speed, 2, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
	Pid_Set(&PID_Shoot[0], 10, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
	Pid_Set(&PID_Shoot[1], 10, 0, 0, 16000, 50, 16000, 16000, 1, 16000, 0, 2);
}

void PID_Init()
{
	Gimbal_PID_Init();
	Chassis_PID_Init();
	Shoot_PID_Init();
}