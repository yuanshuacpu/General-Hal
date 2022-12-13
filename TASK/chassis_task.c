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
 *@Description:	底盘任务
 *@Param:       形参
 *@Return:	  	返回值
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
		// 跟随模式
		case CHASSIS_MODE_FOLLOW:
			Chassis_Follow_Control();
			break;
		// 陀螺模式or扭腰模式
		case CHASSIS_MODE_ROTATE:
			//	Chassis_Rotate_Control();
			Chassis_Twist_Control();
			break;
		// 无力模式
		case CHASSIS_MODE_STOP:
			Chassis_Stop_Control();
			break;
		// 打符模式
		case CHASSIS_MODE_RUNE:
			Chassis_Stop_Control();
			break;
		default:
			break;
		}

		// 底盘速度解算
		Chassis_Speed_Calc(Chassis_Speed.vx, Chassis_Speed.vy, Chassis_Speed.vw);

		Angle_Reverse_Handle();

		// 底盘电容控制
		Chassis_SuperCap_Control();

		// 电机目标电流赋值
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

		// CAN数据更新发送
		osEventFlagsSet(EventCommunicationHandle, CHASSIS_SIGNAL); // 底盘事件组置位
		osDelayUntil(TimeIncrement_chassis + chassis_task_speed);
		TimeIncrement_chassis = osKernelGetTickCount();
	}
}

/**
 *@Function:	底盘电容策略控制
 *@Description:
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_SuperCap_Control(void)
{
	// 安装了电容
	if (SuperCap_Signal > 0)
	{
		// 雾列控制电容
		if (SuperCap_Info.id == 0x211)
		{
			Chassis_SuperCap_Wulie_Control();
		}
		// 自制超级电容
		else if (SuperCap_Info.id == 0x300)
		{
			Chassis_SuperCap_HomeMade_Control();
		}
	}
	// 未接入电容，使用裁判系统数据
	else
	{
		// 开启超速模式，未使用电容情况下慎用
		// 关闭电容模式下
		if (IS_SUPER_OFF)
		{
			Chassis_Power_Limit();
		}
	}
}
/**
 *@Function:	底盘功率限制
 *@Description:
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_Power_Limit()
{
	// 底盘功率控制
	if (Referee_Inf.power_heat_data.chassis_power_buffer < 60)
	{
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= Referee_Inf.power_heat_data.chassis_power_buffer / 60.0f;
		}
	}
}

/**
 *@Function:	底盘雾列电容控制策略
 *@Description:
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_SuperCap_Wulie_Control()
{
	static float power_limit = 1.0; // 功率限制系数
	static uint8_t power_flag = 0;	// 超速标志

	// 关闭电容开关
	if (IS_SUPER_OFF)
	{
		power_flag = 0;
		Super_Allow_Flag = 1;
	}
	// 开启超速模式
	else if (Super_Allow_Flag)
	{
		power_flag = 1;
	}
	else
	{
		power_flag = 0;
	}

	// 若电容电压低于16v强制限速
	if (SuperCap_Info.Low_Filter_Vot < 16.0f)
	{
		Super_Allow_Flag = 0;
		power_flag = 0;
	}

	// 限速状态
	if (power_flag == 0)
	{
		// 底盘功率控制系数
		power_limit = (SuperCap_Info.Low_Filter_Vot - 10.0f) / (SuperCap_Info.InputVot - 10.0f);
		// 系数限幅
		if (power_limit >= 1.0f)
		{
			power_limit = 1.0f;
		}
		else if (power_limit < 0)
		{
			power_limit = 0;
		}
		// 速度限制
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= power_limit;
		}
	}
	// 超速状态，不限制速度上限（也可以对速度进行提升）
	else if (power_flag == 1)
	{
		// 修改power_limit对速度上限进行提升，参考Chassis_Speed_Calc(float vx,float vy,float vw)
		power_limit = 1;
		for (int i = 0; i < 4; i++)
		{
			Chassis_Speed.wheel_speed[i] *= power_limit;
		}
	}
}
/**
 *@Function:	底盘自制电容控制策略
 *@Description:
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_SuperCap_HomeMade_Control()
{
	// 电容低于15v后不允许开启电容模式
	if (SuperCap_Info.CapVot < 15.0f)
	{
		Super_Allow_Flag = 0;
	}
	// 关闭电容模式下限制底盘功率
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
	// 手动开启超级电容模式，且电压允许下
	else if (Super_Allow_Flag)
	{
		time_delay++;
		if (time_delay < 60 || SuperCap_Info.InputCurrent == 0)
		//			if(SuperCap_Info.InputCurrent == 0) //如果电容没有开始供电就继续限制功率 防止电容开启有延迟
		{
			Chassis_Power_Limit();
		}

		power_relay = 1;
	}
	// 手动开启超级电容模式，在电压不允许的情况下，此时强制
	// 切回底盘限速模式，并且需要手动关闭超级电容模式后才能再次开启
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

// 返回Critical值
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
// 返回当前真实角度值
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

// 速度方向标志位
int Init_Dir = 1;

/**
 *@Function:		Angle_Reverse_Handle()
 *@Description:	角度反转处理
 *@Param:       形参
 *@Return:	  	返回值
 */
void Angle_Reverse_Handle()
{
	/*
	实现跟随状态时最小旋转时间，即头尾合用
*/

	// 角度跳变
	if (abs(CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Angle_Last) > 4000)
	{
		if (CAN_Gimbal[0].Critical_MechAngle == Encoder_Data_Cal(&CAN_Gimbal[0], Gimbal_Init_Processing_Data.Origin_Init_Yaw_Angle))
		{
			// 将反面改为正前方
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
			// 将反面改为正前方
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
 *@Description:	底盘跟随控制
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_Follow_Control()
{
	Chassis_Rotate_Base_Speed = 0;
	// 战车速度分量赋值
	Chassis_Speed.vx = rc.vx + keyboard.vx;
	Chassis_Speed.vy = rc.vy + keyboard.vy;

	// Init_Angle = Angle_Correct(Origin_Init_Yaw_Angle);

	Chassis_Speed.vw = Pid_Calc(&PID_Chassis_Omega, (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Init_Angle) / 8192.0 * 360.0f, 0); // 云台Yaw轴相对角PID 输出旋转速度分量

	Chassis_Speed.vx *= Init_Dir;
	Chassis_Speed.vy *= Init_Dir;

	// 旋转速度死区，减少静止底盘轮系抖动
	if (fabs(Chassis_Speed.vw) < 200)
		Chassis_Speed.vw = 0;
}

/**
 *@Function:		Chassis_Rotate_Control()
 *@Description:	底盘陀螺控制
 *@Param:       形参
 *@Return:	  	返回值
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
 *@Description:	底盘扭腰控制
 *@Param:       形参
 *@Return:	  	返回值
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
	Chassis_Speed.vw = Pid_Calc(&PID_Chassis_Omega, (CAN_Gimbal[0].Current_MechAngle - Gimbal_Init_Processing_Data.Init_Angle + Twist_position) / 8192.0 * 360.0f, 0); // 云台Yaw轴相对角PID 输出旋转速度分量

	Twist_position += Twist_Dir * Twist_speed;
	if ((Twist_position > Twist_position_max) || (Twist_position < Twist_position_min))
		Twist_Dir = -Twist_Dir;
}
/**
 *@Function:		Chassis_Stop_Control()
 *@Description:	底盘无力控制
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_Stop_Control()
{
	Chassis_Rotate_Base_Speed = 0;

	Chassis_Speed.vx = 0;
	Chassis_Speed.vy = 0;
	Chassis_Speed.vw = 0;
}

// 轮速最大值限制
void Chassis_Max_Limit()
{
	float Speed_Max = 0;

	// 速度限幅调整
	// 最大值寻找
	for (int i = 0; i < 4; i++)
	{
		if (abs(Chassis_Speed.wheel_speed[i]) > Speed_Max)
		{
			Speed_Max = abs(Chassis_Speed.wheel_speed[i]);
		}
	}
	// 最大轮速限制
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
 *@Description:	底盘速度解算 vx为前后运动速度，vy为左右平移运动速度，vw为旋转速度
 *@Param:       形参
 *@Return:	  	返回值
 */
void Chassis_Speed_Calc(float vx, float vy, float vw)
{
	// 轮速解算
	Chassis_Speed.wheel_speed[0] = +vx + vy + vw;
	Chassis_Speed.wheel_speed[1] = -vx + vy + vw;
	Chassis_Speed.wheel_speed[2] = -vx - vy + vw;
	Chassis_Speed.wheel_speed[3] = +vx - vy + vw;

	// 最大轮速限制
	Chassis_Max_Limit();
}
