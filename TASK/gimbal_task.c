#include "gimbal_task.h"
// 1000/6hz
void gimbal_task(void *p_arg);


void Gimbal_Stop_Control(void);
void Gimbal_Follow_Control(void);
void Gimbal_AutoAim_Control(void);
void Gimbal_Mode_Choose(void);
void Gimbal_Keep_Control(void);
/**
 *@Function:		gimbal_task(void *p_arg)
 *@Description:	云台任务
 *@Param:       形参
 *@Return:	  	返回值
 */
extern int gimbal_task_speed;
void gimbal_task(void *p_arg)
{
	static uint32_t TimeIncrement_gimbal;
	TimeIncrement_gimbal = osKernelGetTickCount();

	while (1)
	{
		switch (Gimbal_Mode)
		{
			// 云台跟随模式
		case GIMBAL_MODE_FOLLOW:
			Gimbal_Follow_Control();
			break;
		// 云台自瞄模式
		case GIMBAL_MODE_AUTOAIM:
			Gimbal_AutoAim_Control();
			break;
		// 云台无力模式
		case GIMBAL_MODE_STOP:
			Gimbal_Stop_Control();
			break;
		// 云台打符模式
		case GIMBAL_MODE_RUNE:
			Gimbal_Stop_Control();
			break;
		default:
			break;
		}

		osEventFlagsSet(EventCommunicationHandle, GIMBAL_SIGNAL);	 // 云台事件组置位
		osEventFlagsSet(EventCommunicationHandle, SHOOT_EXE_SIGNAL); // 发射任务处理置位

		osDelayUntil(TimeIncrement_gimbal + gimbal_task_speed);
		TimeIncrement_gimbal = osKernelGetTickCount();
	}
}

// 云台模式获取
void Gimbal_Mode_Choose(void)
{
	// 左上右上时上云台可遥控器操作
	if (DBUS.RC.Switch_Left == RC_SW_UP && DBUS.RC.Switch_Right == RC_SW_UP)
	{
		Gimbal_Mode = GIMBAL_MODE_FOLLOW;
	}

	// 左中右上，上云台自瞄模式
	else if (DBUS.RC.Switch_Left == RC_SW_MID && DBUS.RC.Switch_Right == RC_SW_UP)
	{
		Gimbal_Mode = GIMBAL_MODE_AUTOAIM;
	}

	// 左中右中，上云台自瞄模式
	else if (DBUS.RC.Switch_Left == RC_SW_MID && DBUS.RC.Switch_Right == RC_SW_MID)
	{
		Gimbal_Mode = GIMBAL_MODE_AUTOAIM;
	}

	// 其它键位下云台为保持模式
	else
	{
		Gimbal_Mode = GIMBAL_MODE_KEEP;
	}

	// 左下无力
	if (DBUS.RC.Switch_Left == RC_SW_DOWN)
	{
		Gimbal_Mode = GIMBAL_MODE_STOP;
	}
}
// 云台手瞄控制
/*
 *@Description：云台跟随控制
 *@param 1：	  参数1
 *@param 2：	  参数2
 *@return:：	  返回值
 */
void Gimbal_Follow_Control()
{
	Gimbal_Init_Processing_Data.Pitch_Aim_Angle -= DBUS.RC.ch3 / 80.0f;

	if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle > Gimbal_Init_Processing_Data.Pitch_Angle_Max)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Max;
	}
	else if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle < Gimbal_Init_Processing_Data.Pitch_Angle_Min)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Min;
	}

	PID_Gimbal_Angle[1].PIDout = Pid_Calc(&PID_Gimbal_Angle[1], CAN_Gimbal[1].Current_MechAngle, Gimbal_Init_Processing_Data.Pitch_Aim_Angle);
	CAN_Gimbal[1].Target_Current = Pid_Calc(&PID_Gimbal_Speed[1], mpu_data.origin_data.gy * 0.1f, PID_Gimbal_Angle[1].PIDout);

	Gimbal_Init_Processing_Data.Yaw_Aim_Angle -= DBUS.RC.ch2 / 80.0f;

	//		if(Yaw_Aim_Angle > Yaw_Angle_Max)
	//		{Yaw_Aim_Angle = Yaw_Angle_Max;}
	//		else if(Yaw_Aim_Angle < Yaw_Angle_Min)
	//		{Yaw_Aim_Angle = Yaw_Angle_Min;}

	PID_Gimbal_Angle[0].PIDout = Pid_Calc(&PID_Gimbal_Angle[0], Yaw_Angle / 360 * 8192, Gimbal_Init_Processing_Data.Yaw_Aim_Angle);
	CAN_Gimbal[0].Target_Current = Pid_Calc(&PID_Gimbal_Speed[0], mpu_data.origin_data.gz * 0.1f, PID_Gimbal_Angle[0].PIDout);
}

// 云台摇摆速度
float Pitch_Speed = 6.0;
float Yaw_Speed = 3.0;
// 云台自瞄控制
/*
 *@Description：云台自瞄控制
 *@param 1：	  参数1
 *@param 2：	  参数2
 *@return:：	  返回值
 */
// void Gimbal_AutoAim_Control()
//{
//	//视觉识别到目标机器人 开启自瞄
//	if(NUC_Data.Armor_Type != 0)
//	{
//		Pitch_Aim_Angle = CAN_Gimbal[1].Current_MechAngle;
//		Pitch_Aim_Angle += NUC_Data.Pitch_Angle;
//
//
//		Yaw_Aim_Angle = CAN_Gimbal[0].Current_MechAngle;
//		Yaw_Aim_Angle -= NUC_Data.Yaw_Angle;
//	}
//
//	//视觉未识别到目标时自动摇头
//	else
//	{
//		Pitch_Aim_Angle += Pitch_Speed;
//		if(Pitch_Aim_Angle > (Pitch_Angle_Max) || Pitch_Aim_Angle < Pitch_Angle_Min)   //顶多仰头到平头不低头
//		{Pitch_Speed = -Pitch_Speed;}
//
//
//		Yaw_Aim_Angle += Yaw_Speed;
//		//if(Yaw_Aim_Angle > Yaw_Angle_Max || Yaw_Aim_Angle < Yaw_Angle_Min)
//		//{Yaw_Speed = -Yaw_Speed;}		                                        				//到头了就反向
//	}
//
//		if(Pitch_Aim_Angle > (Pitch_Angle_Max))
//		{Pitch_Aim_Angle = (Pitch_Angle_Max);}
//		else if(Pitch_Aim_Angle < Pitch_Angle_Min)
//		{Pitch_Aim_Angle = Pitch_Angle_Min;}
//
////
////		if(Yaw_Aim_Angle > Yaw_Angle_Max)
////		{Yaw_Aim_Angle = Yaw_Angle_Max;}
////		else if(Yaw_Aim_Angle < Yaw_Angle_Min)
////		{Yaw_Aim_Angle = Yaw_Angle_Min;}
//
//
//		PID_Gimbal_Angle[1].PIDout = Pid_Calc(&PID_Gimbal_Angle[1],CAN_Gimbal[1].Current_MechAngle,Pitch_Aim_Angle);
//		CAN_Gimbal[1].Target_Current = Pid_Calc(&PID_Gimbal_Speed[1],mpu_data.gy*0.1f,PID_Gimbal_Angle[1].PIDout);
//
//		PID_Gimbal_Angle[0].PIDout = Pid_Calc(&PID_Gimbal_Angle[0],Yaw_Angle/360.0*8192,Yaw_Aim_Angle);
//		CAN_Gimbal[0].Target_Current = Pid_Calc(&PID_Gimbal_Speed[0],mpu_data.gz*0.1f,PID_Gimbal_Angle[0].PIDout);
//}
void Gimbal_AutoAim_Control()
{
	Gimbal_Init_Processing_Data.Pitch_Aim_Angle += DBUS.RC.ch3 / 50.0f - DBUS.PC.Y / 100.0f;
	Gimbal_Init_Processing_Data.Pitch_Aim_Angle += NUC_Data.Pitch_Angle / 1.0f;

	if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle > Gimbal_Init_Processing_Data.Pitch_Angle_Max)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Max;
	}
	else if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle <Gimbal_Init_Processing_Data.Pitch_Angle_Min)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Min;
	}

	PID_Gimbal_Angle[1].PIDout = Pid_Calc(&PID_Gimbal_Angle[1], CAN_Gimbal[1].Current_MechAngle, Gimbal_Init_Processing_Data.Pitch_Aim_Angle);
	CAN_Gimbal[1].Target_Current = Pid_Calc(&PID_Gimbal_Speed[1], -mpu_data.origin_data.gy * 0.1f, PID_Gimbal_Angle[1].PIDout);

	Gimbal_Init_Processing_Data.Yaw_Aim_Angle = Yaw_Angle;
	Gimbal_Init_Processing_Data.Yaw_Aim_Angle -= DBUS.RC.ch2 / 600.0f + DBUS.PC.X / 100.0f;
	Gimbal_Init_Processing_Data.Yaw_Aim_Angle += NUC_Data.Yaw_Angle;

	PID_Gimbal_Angle[0].PIDout = Pid_Calc(&PID_Gimbal_Angle[0], Yaw_Angle, Gimbal_Init_Processing_Data.Yaw_Aim_Angle);
	CAN_Gimbal[0].Target_Current = Pid_Calc(&PID_Gimbal_Speed[0], mpu_data.origin_data.gz * 0.1f, PID_Gimbal_Angle[0].PIDout);
}
/*
 *@Description：云台保持控制突然停下来保持在固定位置
 *@param 1：	  参数1
 *@param 2：	  参数2
 *@return:：	  返回值
 */
void Gimbal_Keep_Control()
{
	Gimbal_Init_Processing_Data.Pitch_Aim_Angle = CAN_Gimbal[1].Current_MechAngle;

	if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle > Gimbal_Init_Processing_Data.Pitch_Angle_Max)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Max;
	}
	else if (Gimbal_Init_Processing_Data.Pitch_Aim_Angle < Gimbal_Init_Processing_Data.Pitch_Angle_Min)
	{
		Gimbal_Init_Processing_Data.Pitch_Aim_Angle = Gimbal_Init_Processing_Data.Pitch_Angle_Min;
	}

	PID_Gimbal_Angle[1].PIDout = Pid_Calc(&PID_Gimbal_Angle[1], CAN_Gimbal[1].Current_MechAngle, Gimbal_Init_Processing_Data.Pitch_Aim_Angle);
	CAN_Gimbal[1].Target_Current = Pid_Calc(&PID_Gimbal_Speed[1], mpu_data.origin_data.gy * 0.1f, PID_Gimbal_Angle[1].PIDout);

	// Yaw_Aim_Angle = CAN_Gimbal[0].Current_MechAngle;

	//		if(Yaw_Aim_Angle > Yaw_Angle_Max)
	//		{Yaw_Aim_Angle = Yaw_Angle_Max;}
	//		else if(Yaw_Aim_Angle < Yaw_Angle_Min)
	//		{Yaw_Aim_Angle = Yaw_Angle_Min;}

	PID_Gimbal_Angle[0].PIDout = Pid_Calc(&PID_Gimbal_Angle[0], Yaw_Angle / 360.0 * 8192.0, Gimbal_Init_Processing_Data.Yaw_Aim_Angle);
	CAN_Gimbal[0].Target_Current = Pid_Calc(&PID_Gimbal_Speed[0], mpu_data.origin_data.gz * 0.1f, PID_Gimbal_Angle[0].PIDout);
}
/*
 *@Description：云台无力控制
 *@param 1：	  参数1
 *@param 2：	  参数2
 *@return:：	  返回值
 */
void Gimbal_Stop_Control()
{
	CAN_Gimbal[0].Target_Current = 0;
	CAN_Gimbal[1].Target_Current = 0;
}
