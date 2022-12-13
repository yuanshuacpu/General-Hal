#include "shoot_task.h"



void Shoot_Stop_Control(void);
void Shoot_Run_Control(void);
void Bullet_Block_Control(void);

/**
 *@Function:		shoot_task(void *p_arg)
 *@Description:	��������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void shoot_task(void *p_arg)
{
	EventBits_t EventValue;

	while (1)
	{
		EventValue = osEventFlagsWait(EventCommunicationHandle, SHOOT_EXE_SIGNAL, osFlagsWaitAll, osWaitForever);
		if (EventValue & SHOOT_EXE_SIGNAL)
		{
			switch (Shoot_Mode)
			{
			// ����ֹͣ
			case SHOOT_MODE_STOP:
				Shoot_Stop_Control();
				break;
			// ��������
			case SHOOT_MODE_RUN:
				Shoot_Run_Control();
				break;

			default:
				break;
			}
		}
		osEventFlagsSet(EventCommunicationHandle, SHOOT_SIGNAL); // �����¼�����λ
	}
}
/**
 *@Function:		Shoot_Stop_Control()
 *@Description:	������������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Shoot_Stop_Control()
{
	Shoot_Cmd = SHOOT_CMD_STOP; // ֹͣ����

	CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger[0], CAN_Trigger.Current_Speed, 0);
	CAN_Shoot[0].Target_Current = Pid_Calc(&PID_Shoot[0], CAN_Shoot[0].Current_Speed, 0);
	CAN_Shoot[1].Target_Current = Pid_Calc(&PID_Shoot[1], CAN_Shoot[1].Current_Speed, 0);
}

volatile float shoot_speed_get = 0;
int Shoot_Speed = 0;

// 7000����30m/s
// 4500����15m/s
int shoot_bullet_speed = 4500;

// ��ת���ڼ�ʱ
static int Block_Time = 0;
// ��תʱ���ʱ
static int Block_Reverse_Time = 0;
int speed_control_count = 0;
float speed_record[6] = {0, 0, 0, 0, 0, 0}; // �����Ϊ��������� ������Ϊƽ������
int speed_minus = 0;						// Ħ���ֽ���

int shoot_counter_record = 0;				// ��¼�ķ�����ӵ���
/**
 *@Function:		Shoot_Run_Control()
 *@Description:	�������п���
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Shoot_Run_Control()
{
	// �����ж�
	//	if(game_robot_state.shooter_id1_17mm_speed_limit == 15)
	//	{
	////		if(shoot_speed_get != real_shoot_data.bullet_speed)
	////		{
	////			if(real_shoot_data.bullet_speed > 14.3)
	////			{
	////				speed_control_count -= ((real_shoot_data.bullet_speed - 14.3)/0.1);
	////			}
	////			else if(real_shoot_data.bullet_speed < 14.0)
	////			{
	////				speed_control_count += ((14.0 - real_shoot_data.bullet_speed)/0.1);
	////			}
	////		}
	////		shoot_bullet_speed = 4350 + speed_control_count * 25;
	////		if(shoot_bullet_speed > 4400)shoot_bullet_speed = 4400;
	////		if(shoot_bullet_speed < 4300)shoot_bullet_speed = 4300;
	//		if(shoot_speed_get != real_shoot_data.bullet_speed)
	//		{
	//			speed_record[0] = speed_record[1];
	//			speed_record[1] = speed_record[2];
	//			speed_record[2] = speed_record[3];
	//			speed_record[3] = speed_record[4];
	//			speed_record[4] = real_shoot_data.bullet_speed;
	//			speed_record[5] = (speed_record[0]+speed_record[1]+speed_record[2]+speed_record[3]+speed_record[4])/5.0;
	//			if(speed_record[5] >= 14.4) //�����ŵ���ƽ��ֵ>=14.6 Ħ���ֽ���
	//			{
	//				speed_minus++;
	//			}
	//		}
	//		shoot_bullet_speed = 4500 - 5*speed_minus;
	//	}
	//	else if(game_robot_state.shooter_id1_17mm_speed_limit == 30)
	//	{
	//		shoot_bullet_speed = 7000;
	//	}
	//	else
	//	{
	//		shoot_bullet_speed = 4500;
	//	}
	shoot_bullet_speed = 7000; // �̶�30m/s������ 7000

	// ����ģʽ
	if (Shoot_Cmd == SHOOT_CMD_ONCE) // && game_robot_state.shooter_id1_17mm_cooling_limit >= power_heat_data.shooter_id1_17mm_cooling_heat + 10*3)
	{

		if (Shoot_Once_Flag)
		{
			if (Block_Reverse_Time == 0)
			{
				PID_Trigger_Angle.PIDout = Pid_Calc(&PID_Trigger_Angle, CAN_Trigger.Total_MechAngle, Shoot_Aim_Angle);
				CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger_Speed, CAN_Trigger.Current_Speed, PID_Trigger_Angle.PIDout);
				if (fabs(PID_Trigger_Angle.CurrentError) < 10)
				{
					Shoot_Once_Flag = 0;
					Shoot_Cmd = SHOOT_CMD_STOP;
				}
			}
			else
			{
				CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger[0], CAN_Trigger.Current_Speed, Shoot_Speed);
			}
		}
	}
	// ����ģʽ
	else if (Shoot_Cmd == SHOOT_CMD_LONG)
	{
		// �жϲ����Ƿ�λ���·�����������һֱ��ס
		if (Trig_Time == 0)
		{
			Shoot_Speed = 0;
			Shoot_Cmd = SHOOT_CMD_STOP;
		}
		else
		{
			// ����Ҫ�󷢵��ȶ�
			// 5500 15m/s
			// 7000 30m/s

			Shoot_Speed = 7000;
		}
	}
	else
	{
		Shoot_Speed = 0;
	}

	// ���俨������
	Bullet_Block_Control();
	shoot_counter_record = shoot_counter;
	// �������ƣ���ֹ���������������ŵ�����������֤����Ƶ�²�������
	heat_remain = Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_limit - Referee_Inf.power_heat_data.shooter_id1_17mm_cooling_heat; // �����ʣ������
	if ((heat_remain <= 10 * heat_control) || ((shoot_counter - shoot_counter_record) >= (heat_remain / 10.0 - heat_control)))			   // ʣ������С����������������������ӵ������������ӵ�
	{
		Shoot_Speed = 0;
	}

	// ���ٻ�ȡ
	shoot_speed_get = Referee_Inf.real_shoot_data.bullet_speed;

	// ������ת�ٴ���
	if (Shoot_Cmd != SHOOT_CMD_ONCE)
		CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger[0], CAN_Trigger.Current_Speed, Shoot_Speed);

	// Ħ����ת�ٴ���
	CAN_Shoot[0].Target_Current = Pid_Calc(&PID_Shoot[0], CAN_Shoot[0].Current_Speed, -shoot_bullet_speed);
	CAN_Shoot[1].Target_Current = Pid_Calc(&PID_Shoot[1], CAN_Shoot[1].Current_Speed, shoot_bullet_speed);
}
/**
 *@Function:		Bullet_Block_Control()
 *@Description:	��������
 *@Param:       �β�
 *@Return:	  	����ֵ
 */
void Bullet_Block_Control()
{

	// �����������ʱ��ⷴת
	if (CAN_Trigger.Current > 10000)
	{
		Block_Time++;
	}

	// ��ת
	if (Block_Reverse_Time >= 1)
	{
		Block_Reverse_Time++;

		// ��תʱ�� 3 * 2ms = 6ms
		if (Block_Reverse_Time > 20)
		{
			Block_Reverse_Time = 0;
			Block_Time = 0;
		}
		else
		{
			// ��תʱ������Сʱ������ת����ֹ��ת��ת
			if (CAN_Trigger.Current > -4000)
			{
				Shoot_Speed = -4000;
			}
			// �����ϴ�ָ���ת
			else
			{
				Block_Reverse_Time = 0;
				Block_Time = 0;
			}
		}
	}
	else
	{
		// ��תʱ��10*�����������ڣ�2ms��= 20ms
		if (Block_Time == 10)
		{
			Block_Reverse_Time = 1;
		}
	}
}
