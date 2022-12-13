#include "shoot_task.h"



void Shoot_Stop_Control(void);
void Shoot_Run_Control(void);
void Bullet_Block_Control(void);

/**
 *@Function:		shoot_task(void *p_arg)
 *@Description:	发射任务
 *@Param:       形参
 *@Return:	  	返回值
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
			// 发射停止
			case SHOOT_MODE_STOP:
				Shoot_Stop_Control();
				break;
			// 发射运行
			case SHOOT_MODE_RUN:
				Shoot_Run_Control();
				break;

			default:
				break;
			}
		}
		osEventFlagsSet(EventCommunicationHandle, SHOOT_SIGNAL); // 发射事件组置位
	}
}
/**
 *@Function:		Shoot_Stop_Control()
 *@Description:	发射无力控制
 *@Param:       形参
 *@Return:	  	返回值
 */
void Shoot_Stop_Control()
{
	Shoot_Cmd = SHOOT_CMD_STOP; // 停止发射

	CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger[0], CAN_Trigger.Current_Speed, 0);
	CAN_Shoot[0].Target_Current = Pid_Calc(&PID_Shoot[0], CAN_Shoot[0].Current_Speed, 0);
	CAN_Shoot[1].Target_Current = Pid_Calc(&PID_Shoot[1], CAN_Shoot[1].Current_Speed, 0);
}

volatile float shoot_speed_get = 0;
int Shoot_Speed = 0;

// 7000——30m/s
// 4500——15m/s
int shoot_bullet_speed = 4500;

// 堵转周期计时
static int Block_Time = 0;
// 反转时间计时
static int Block_Reverse_Time = 0;
int speed_control_count = 0;
float speed_record[6] = {0, 0, 0, 0, 0, 0}; // 第五个为最近的射速 第六个为平均射速
int speed_minus = 0;						// 摩擦轮降速

int shoot_counter_record = 0;				// 记录的发射的子弹数
/**
 *@Function:		Shoot_Run_Control()
 *@Description:	发射运行控制
 *@Param:       形参
 *@Return:	  	返回值
 */
void Shoot_Run_Control()
{
	// 射速判断
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
	//			if(speed_record[5] >= 14.4) //最近五颗弹速平均值>=14.6 摩擦轮降速
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
	shoot_bullet_speed = 7000; // 固定30m/s的射速 7000

	// 单发模式
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
	// 连发模式
	else if (Shoot_Cmd == SHOOT_CMD_LONG)
	{
		// 判断拨杆是否位于下方或者鼠标左键一直按住
		if (Trig_Time == 0)
		{
			Shoot_Speed = 0;
			Shoot_Cmd = SHOOT_CMD_STOP;
		}
		else
		{
			// 连发要求发弹稳定
			// 5500 15m/s
			// 7000 30m/s

			Shoot_Speed = 7000;
		}
	}
	else
	{
		Shoot_Speed = 0;
	}

	// 发射卡弹控制
	Bullet_Block_Control();
	shoot_counter_record = shoot_counter;
	// 热量限制（防止超热量）留出三颗弹丸余量，保证高射频下不超热量
	heat_remain = Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_limit - Referee_Inf.power_heat_data.shooter_id1_17mm_cooling_heat; // 计算出剩余热量
	if ((heat_remain <= 10 * heat_control) || ((shoot_counter - shoot_counter_record) >= (heat_remain / 10.0 - heat_control)))			   // 剩余热量小于留出的热量或者射出的子弹大于留出的子弹
	{
		Shoot_Speed = 0;
	}

	// 射速获取
	shoot_speed_get = Referee_Inf.real_shoot_data.bullet_speed;

	// 拨弹盘转速处理
	if (Shoot_Cmd != SHOOT_CMD_ONCE)
		CAN_Trigger.Target_Current = Pid_Calc(&PID_Trigger[0], CAN_Trigger.Current_Speed, Shoot_Speed);

	// 摩擦轮转速处理
	CAN_Shoot[0].Target_Current = Pid_Calc(&PID_Shoot[0], CAN_Shoot[0].Current_Speed, -shoot_bullet_speed);
	CAN_Shoot[1].Target_Current = Pid_Calc(&PID_Shoot[1], CAN_Shoot[1].Current_Speed, shoot_bullet_speed);
}
/**
 *@Function:		Bullet_Block_Control()
 *@Description:	卡弹控制
 *@Param:       形参
 *@Return:	  	返回值
 */
void Bullet_Block_Control()
{

	// 电机电流过大时检测反转
	if (CAN_Trigger.Current > 10000)
	{
		Block_Time++;
	}

	// 反转
	if (Block_Reverse_Time >= 1)
	{
		Block_Reverse_Time++;

		// 反转时间 3 * 2ms = 6ms
		if (Block_Reverse_Time > 20)
		{
			Block_Reverse_Time = 0;
			Block_Time = 0;
		}
		else
		{
			// 反转时电流较小时才允许反转，防止反转堵转
			if (CAN_Trigger.Current > -4000)
			{
				Shoot_Speed = -4000;
			}
			// 电流较大恢复正转
			else
			{
				Block_Reverse_Time = 0;
				Block_Time = 0;
			}
		}
	}
	else
	{
		// 堵转时间10*发射任务周期（2ms）= 20ms
		if (Block_Time == 10)
		{
			Block_Reverse_Time = 1;
		}
	}
}
