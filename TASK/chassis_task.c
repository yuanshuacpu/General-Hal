#include "chassis_task.h"
// 1000/3hz
void Chassis_Speed_Calc(float vx, float vy, float vw);
void Chassis_Follow_Control(void);
void Chassis_Stop_Control(void);
void Chassis_Rotate_Control(void);
void Chassis_Mode_Choose(void);
void Chassis_Twist_Control(void);

void Get_Chassis_RC(void);

void Chassis_Rudder_Nearby_Cal(float vx, float vy, float vw);

void Angle_Reverse_Handle(void);

#define ANGLE_TO_RAD 0.01745329251994329576923690768489f

#define IS_SUPER_OFF DBUS.RC.ch4 > -600

void chassis_task(void *p_arg);

Chassis_Speed_Typedef Chassis_Speed;

void Chassis_Power_Limit(void);
void Chassis_SuperCap_Wulie_Control(void);
void Chassis_SuperCap_HomeMade_Control(void);
void Chassis_SuperCap_Control(void);

int Super_Allow_Flag = 0;
int time_delay = 0;
int time_delay1 = 0;
int time_delay2 = 0;

/**
 *@Function:		chassis_task(void *p_arg)
 *@Description:	��������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
extern int chassis_task_speed;
void chassis_task(void *p_arg)
{
	static uint32_t TimeIncrement_chassis;
	TimeIncrement_chassis = osKernelGetTickCount();

	while (1)
	{

		switch (Chassis_Mode)
		{
		// ����ģʽ
		case CHASSIS_MODE_FOLLOW:
			Chassis_Follow_Control();
			break;
		// ����ģʽorŤ��ģʽ
		case CHASSIS_MODE_ROTATE:
			//	Chassis_Rotate_Control();
			Chassis_Twist_Control();
			break;
		// ����ģʽ
		case CHASSIS_MODE_STOP:
			Chassis_Stop_Control();
			break;
		// ���ģʽ
		case CHASSIS_MODE_RUNE:
			Chassis_Stop_Control();
			break;
		default:
			break;
		}

		// �����ٶȽ���
		Chassis_Speed_Calc(Chassis_Speed.vx, Chassis_Speed.vy, Chassis_Speed.vw);

		Angle_Reverse_Handle();

		// ���̵��ݿ���
		Chassis_SuperCap_Control();

		// ���Ŀ�������ֵ
		for (int i = 0; i < 4; i++)
		{
			CAN_Chassis[i].Target_Current = Pid_Calc(&PID_Chassis[i], CAN_Chassis[i].Current_Speed, Chassis_Speed.wheel_speed[i]);
		}

		if (Chassis_Mode == CHASSIS_MODE_STOP)
		{
			for (int i = 0; i < 4; i++)
			{
				CAN_Chassis[i].Target_Current = Pid_Calc(&PID_Chassis[i], CAN_Chassis[i].Current_Speed, 0);
			}
		}

		// CAN���ݸ��·���
		osEventFlagsSet(EventCommunicationHandle, CHASSIS_SIGNAL); // �����¼�����λ
		osDelayUntil(TimeIncrement_chassis + chassis_task_speed);
		TimeIncrement_chassis = osKernelGetTickCount();
	}
}

/**
 *@Function:	���̵��ݲ��Կ���
 *@Description:
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_SuperCap_Control(void)
{
	// ��װ�˵���
	if (SuperCap_Signal > 0)
	{
		// ���п��Ƶ���
		if (SuperCap_Info.id == 0x211)
		{
			Chassis_SuperCap_Wulie_Control();
		}
		// ���Ƴ�������
		else if (SuperCap_Info.id == 0x300)
		{
			Chassis_SuperCap_HomeMade_Control();
		}
	}
	// δ������ݣ�ʹ�ò���ϵͳ����
	else
	{
		// ��������ģʽ��δʹ�õ������������
		// �رյ���ģʽ��
		if (IS_SUPER_OFF)
		{
			Chassis_Power_Limit();
		}
	}
}
/**
 *@Function:	���̹�������
 *@Description:
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_Power_Limit()
{
	// ���̹��ʿ���
	if (Referee_Inf.power_heat_data.chassis_power_buffer < 60)
	{
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= Referee_Inf.power_heat_data.chassis_power_buffer / 60.0f;
		}
	}
}

/**
 *@Function:	�������е��ݿ��Ʋ���
 *@Description:
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_SuperCap_Wulie_Control()
{
	static float power_limit = 1.0; // ��������ϵ��
	static uint8_t power_flag = 0;	// ���ٱ�־

	// �رյ��ݿ���
	if (IS_SUPER_OFF)
	{
		power_flag = 0;
		Super_Allow_Flag = 1;
	}
	// ��������ģʽ
	else if (Super_Allow_Flag)
	{
		power_flag = 1;
	}
	else
	{
		power_flag = 0;
	}

	// �����ݵ�ѹ����16vǿ������
	if (SuperCap_Info.Low_Filter_Vot < 16.0f)
	{
		Super_Allow_Flag = 0;
		power_flag = 0;
	}

	// ����״̬
	if (power_flag == 0)
	{
		// ���̹��ʿ���ϵ��
		power_limit = (SuperCap_Info.Low_Filter_Vot - 10.0f) / (SuperCap_Info.InputVot - 10.0f);
		// ϵ���޷�
		if (power_limit >= 1.0f)
		{
			power_limit = 1.0f;
		}
		else if (power_limit < 0)
		{
			power_limit = 0;
		}
		// �ٶ�����
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= power_limit;
		}
	}
	// ����״̬���������ٶ����ޣ�Ҳ���Զ��ٶȽ���������
	else if (power_flag == 1)
	{
		// �޸�power_limit���ٶ����޽����������ο�Chassis_Speed_Calc(float vx,float vy,float vw)
		power_limit = 1;
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= power_limit;
		}
	}
}
/**
 *@Function:	�������Ƶ��ݿ��Ʋ���
 *@Description:
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_SuperCap_HomeMade_Control()
{
	// ���ݵ���15v������������ģʽ
	if (SuperCap_Info.CapVot < 15.0f)
	{
		Super_Allow_Flag = 0;
	}
	// �رյ���ģʽ�����Ƶ��̹���
	if (IS_SUPER_OFF)
	{
		Super_Allow_Flag = 1;

		time_delay = 0;
		Chassis_Power_Limit();
		time_delay1++;
		if (time_delay1 > 50)
		{
			power_relay = 0;
			time_delay1 = 0;
		}
	}
	// �ֶ�������������ģʽ���ҵ�ѹ������
	else if (Super_Allow_Flag)
	{
		time_delay++;
		if (time_delay < 60 || SuperCap_Info.InputCurrent == 0)
		//			if(SuperCap_Info.InputCurrent == 0) //�������û�п�ʼ����ͼ������ƹ��� ��ֹ���ݿ������ӳ�
		{
			Chassis_Power_Limit();
		}

		power_relay = 1;
	}
	// �ֶ�������������ģʽ���ڵ�ѹ�����������£���ʱǿ��
	// �лص�������ģʽ��������Ҫ�ֶ��رճ�������ģʽ������ٴο���
	else
	{
		time_delay = 0;
		Chassis_Power_Limit();
		time_delay2++;
		if (time_delay2 > 50)
		{
			power_relay = 0;
			time_delay2 = 0;
		}
	}
}

// ����Criticalֵ
int Encoder_Data_Cal(CAN_Data_TypeDef *encoder_data, short init_angle)
{
	if (init_angle + 4096 > 8191)
	{
		return init_angle + 4096 - 8192;
	}
	else
	{
		return init_angle + 4096;
	}
}
// ���ص�ǰ��ʵ�Ƕ�ֵ
int Angle_Correct(int angle_to_resize)
{
	if (angle_to_resize < CAN_Gimbal[0].Critical_MechAngle) // 7833
	{
		return angle_to_resize + 8192;
	}
	else
	{
		return angle_to_resize;
	}
}

// �ٶȷ����־λ
int Init_Dir = 1;

/**
 *@Function:		Angle_Reverse_Handle()
 *@Description:	�Ƕȷ�ת����
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Angle_Reverse_Handle()
{
	/*
	ʵ�ָ���״̬ʱ��С��תʱ�䣬��ͷβ����
*/

	// �Ƕ�����
	if (abs(CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Angle_Last) > 4000)
	{
		if (CAN_Gimbal[0].Critical_MechAngle == Encoder_Data_Cal(&CAN_Gimbal[0], Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle))
		{
			// �������Ϊ��ǰ��
			Encoder_Data_Process(&CAN_Gimbal[0], Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle + 4096);
			if (CAN_Gimbal[0].Origin_MechAngle < CAN_Gimbal[0].Critical_MechAngle)
			{
				CAN_Gimbal[0].Current_MechAngle = CAN_Gimbal[0].Origin_MechAngle + 8192;
			}
			else
			{
				CAN_Gimbal[0].Current_MechAngle = CAN_Gimbal[0].Origin_MechAngle;
			}

			Gimbal_Init_Processing_Data.Init_Angle = Angle_Correct(Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle + 4096);
		}
		else
		{
			// �������Ϊ��ǰ��
			Encoder_Data_Process(&CAN_Gimbal[0], Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle);
			if (CAN_Gimbal[0].Origin_MechAngle < CAN_Gimbal[0].Critical_MechAngle)
			{
				CAN_Gimbal[0].Current_MechAngle = CAN_Gimbal[0].Origin_MechAngle + 8192;
			}
			else
			{
				CAN_Gimbal[0].Current_MechAngle = CAN_Gimbal[0].Origin_MechAngle;
			}

			Gimbal_Init_Processing_Data.Init_Angle = Angle_Correct(Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle);
		}

		Init_Dir = -Init_Dir;
	}

	Gimbal_Init_Processing_Data.Angle_Last = CAN_Gimbal[0].Current_MechAngle;
}
int Chassis_Rotate_Base_Speed = 0;
/**
 *@Function:		Chassis_Follow_Control()
 *@Description:	���̸������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_Follow_Control()
{
	Chassis_Rotate_Base_Speed = 0;
	// ս���ٶȷ�����ֵ
	Chassis_Speed.vx = rc.vx + keyboard.vx;
	Chassis_Speed.vy = rc.vy + keyboard.vy;

	// Init_Angle = Angle_Correct(Origin_Init_Yaw_Angle);

	Chassis_Speed.vw = Pid_Calc(&PID_Chassis_Omega, (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Init_Angle) / 8192.0 * 360.0f, 0); // ��̨Yaw����Խ�PID �����ת�ٶȷ���

	Chassis_Speed.vx *= Init_Dir;
	Chassis_Speed.vy *= Init_Dir;

	// ��ת�ٶ����������پ�ֹ������ϵ����
	if (fabs(Chassis_Speed.vw) < 200)
		Chassis_Speed.vw = 0;
}

/**
 *@Function:		Chassis_Rotate_Control()
 *@Description:	�������ݿ���
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
extern float rotate_speed;

void Chassis_Rotate_Control()
{
	Chassis_Rotate_Base_Speed = rotate_speed;

	static double Rotate_Angle = 0;

	Rotate_Angle = (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle) / 8192.0 * 360.0f;
	if (Rotate_Angle <= 0)
	{
		Rotate_Angle = Rotate_Angle + 360.0f;
	}

	Chassis_Speed.vx = +(rc.vy) * sin(Rotate_Angle * ANGLE_TO_RAD) + (rc.vx) * cos(Rotate_Angle * ANGLE_TO_RAD);
	Chassis_Speed.vy = +(rc.vy) * cos(Rotate_Angle * ANGLE_TO_RAD) - (rc.vx) * sin(Rotate_Angle * ANGLE_TO_RAD);
	Chassis_Speed.vw = Chassis_Rotate_Base_Speed;
}
/**
 *@Function:		Chassis_Twist_Control()
 *@Description:	����Ť������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
extern float Twist_speed;
float Twist_position_max = 2048;
float Twist_position_min = -2048;
float Twist_position = 0;
float Twist_Dir = 1;
void Chassis_Twist_Control(void)
{

	static double Rotate_Angle = 0;

	Rotate_Angle = (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle) / 8192.0 * 360.0f;
	if (Rotate_Angle <= 0)
	{
		Rotate_Angle = Rotate_Angle + 360.0f;
	}

	Chassis_Speed.vx = +(rc.vy) * sin(Rotate_Angle * ANGLE_TO_RAD) + (rc.vx) * cos(Rotate_Angle * ANGLE_TO_RAD);
	Chassis_Speed.vy = +(rc.vy) * cos(Rotate_Angle * ANGLE_TO_RAD) - (rc.vx) * sin(Rotate_Angle * ANGLE_TO_RAD);
	Chassis_Speed.vw = Pid_Calc(&PID_Chassis_Omega, (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Init_Angle + Twist_position) / 8192.0 * 360.0f, 0); // ��̨Yaw����Խ�PID �����ת�ٶȷ���

	Twist_position += Twist_Dir * Twist_speed;
	if ((Twist_position > Twist_position_max) || (Twist_position < Twist_position_min))
		Twist_Dir = -Twist_Dir;
}
/**
 *@Function:		Chassis_Stop_Control()
 *@Description:	������������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_Stop_Control()
{
	Chassis_Rotate_Base_Speed = 0;

	Chassis_Speed.vx = 0;
	Chassis_Speed.vy = 0;
	Chassis_Speed.vw = 0;
}

// �������ֵ����
void Chassis_Max_Limit()
{
	float Speed_Max = 0;

	// �ٶ��޷�����
	// ���ֵѰ��
	for (int i = 0; i < 4; i++)
	{
		if (abs(Chassis_Speed.wheel_speed[i]) > Speed_Max)
		{
			Speed_Max = abs(Chassis_Speed.wheel_speed[i]);
		}
	}
	// �����������
	if (Speed_Max > MAX_WHEEL_SPEED)
	{
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= MAX_WHEEL_SPEED / Speed_Max;
		}
	}
}

/**
 *@Function:		Chassis_Speed_Calc(float vx,float vy,float vw)
 *@Description:	�����ٶȽ��� vxΪǰ���˶��ٶȣ�vyΪ����ƽ���˶��ٶȣ�vwΪ��ת�ٶ�
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Chassis_Speed_Calc(float vx, float vy, float vw)
{
	// ���ٽ���
	Chassis_Speed.wheel_speed[0] = +vx + vy + vw;
	Chassis_Speed.wheel_speed[1] = -vx + vy + vw;
	Chassis_Speed.wheel_speed[2] = -vx - vy + vw;
	Chassis_Speed.wheel_speed[3] = +vx - vy + vw;

	// �����������
	Chassis_Max_Limit();
}
