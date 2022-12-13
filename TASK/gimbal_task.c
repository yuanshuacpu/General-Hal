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
 *@Description:	��̨����
 *@Param:       �β�
 *@Return:	  	����ֵ
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
			// ��̨����ģʽ
		case GIMBAL_MODE_FOLLOW:
			Gimbal_Follow_Control();
			break;
		// ��̨����ģʽ
		case GIMBAL_MODE_AUTOAIM:
			Gimbal_AutoAim_Control();
			break;
		// ��̨����ģʽ
		case GIMBAL_MODE_STOP:
			Gimbal_Stop_Control();
			break;
		// ��̨���ģʽ
		case GIMBAL_MODE_RUNE:
			Gimbal_Stop_Control();
			break;
		default:
			break;
		}

		osEventFlagsSet(EventCommunicationHandle, GIMBAL_SIGNAL);	 // ��̨�¼�����λ
		osEventFlagsSet(EventCommunicationHandle, SHOOT_EXE_SIGNAL); // ������������λ

		osDelayUntil(TimeIncrement_gimbal + gimbal_task_speed);
		TimeIncrement_gimbal = osKernelGetTickCount();
	}
}

// ��̨ģʽ��ȡ
void Gimbal_Mode_Choose(void)
{
	// ��������ʱ����̨��ң��������
	if (DBUS.RC.Switch_Left == RC_SW_UP && DBUS.RC.Switch_Right == RC_SW_UP)
	{
		Gimbal_Mode = GIMBAL_MODE_FOLLOW;
	}

	// �������ϣ�����̨����ģʽ
	else if (DBUS.RC.Switch_Left == RC_SW_MID && DBUS.RC.Switch_Right == RC_SW_UP)
	{
		Gimbal_Mode = GIMBAL_MODE_AUTOAIM;
	}

	// �������У�����̨����ģʽ
	else if (DBUS.RC.Switch_Left == RC_SW_MID && DBUS.RC.Switch_Right == RC_SW_MID)
	{
		Gimbal_Mode = GIMBAL_MODE_AUTOAIM;
	}

	// ������λ����̨Ϊ����ģʽ
	else
	{
		Gimbal_Mode = GIMBAL_MODE_KEEP;
	}

	// ��������
	if (DBUS.RC.Switch_Left == RC_SW_DOWN)
	{
		Gimbal_Mode = GIMBAL_MODE_STOP;
	}
}
// ��̨�������
/*
 *@Description����̨�������
 *@param 1��	  ����1
 *@param 2��	  ����2
 *@return:��	  ����ֵ
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

// ��̨ҡ���ٶ�
float Pitch_Speed = 6.0;
float Yaw_Speed = 3.0;
// ��̨�������
/*
 *@Description����̨�������
 *@param 1��	  ����1
 *@param 2��	  ����2
 *@return:��	  ����ֵ
 */
// void Gimbal_AutoAim_Control()
//{
//	//�Ӿ�ʶ��Ŀ������� ��������
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
//	//�Ӿ�δʶ��Ŀ��ʱ�Զ�ҡͷ
//	else
//	{
//		Pitch_Aim_Angle += Pitch_Speed;
//		if(Pitch_Aim_Angle > (Pitch_Angle_Max) || Pitch_Aim_Angle < Pitch_Angle_Min)   //������ͷ��ƽͷ����ͷ
//		{Pitch_Speed = -Pitch_Speed;}
//
//
//		Yaw_Aim_Angle += Yaw_Speed;
//		//if(Yaw_Aim_Angle > Yaw_Angle_Max || Yaw_Aim_Angle < Yaw_Angle_Min)
//		//{Yaw_Speed = -Yaw_Speed;}		                                        				//��ͷ�˾ͷ���
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
 *@Description����̨���ֿ���ͻȻͣ���������ڹ̶�λ��
 *@param 1��	  ����1
 *@param 2��	  ����2
 *@return:��	  ����ֵ
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
 *@Description����̨��������
 *@param 1��	  ����1
 *@param 2��	  ����2
 *@return:��	  ����ֵ
 */
void Gimbal_Stop_Control()
{
	CAN_Gimbal[0].Target_Current = 0;
	CAN_Gimbal[1].Target_Current = 0;
}
