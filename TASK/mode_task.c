#include "mode_task.h"

void Get_Chassis_Mode(void);
void Get_Gimbal_Mode(void);
void Get_Chassis_RC(void);
void Get_Chassis_Keyboard(void);
void Get_Shoot_Mode(void);
void Get_Cover_Mode(void);
void Get_Keyboard_Mode(void);

/**
 *@Function:		mode_task(void *p_arg)
 *@Description:	模式切换任务
 *@Param:       形参
 *@Return:	  	返回值
 */
void mode_task(void *p_arg)
{
	static uint32_t TimeIncrement_mode;
	TimeIncrement_mode = osKernelGetTickCount();

	while (1)
	{
		// 信号衰减
		CAN1_Signal--;
		CAN2_Signal--;
		SuperCap_Signal--;

		DBUS.DR16_Signal--; // 信号量递减，在此之前读值应该不为0,否则视为遥控器断电

		// 遥控器断连，机器人关闭控制
		if (DBUS.DR16_Signal <= 0)
		{
			Chassis_Mode = CHASSIS_MODE_STOP;
			Gimbal_Mode = GIMBAL_MODE_STOP;
			Shoot_Mode = SHOOT_MODE_STOP;
			DBUS.DR16_Signal = 0;
		}
		// 遥控器连接中，正常控制
		else
		{
			// 底盘速度获取
			Get_Chassis_RC();
			Get_Chassis_Keyboard();

			// 各功能模式状态更新
			Get_Keyboard_Mode();
			Get_Gimbal_Mode();
			Get_Chassis_Mode();
			Get_Shoot_Mode();
			Get_Cover_Mode();
		}

		// CAN2断连时摩擦轮停
		if (CAN2_Signal <= 0)
		{
			Shoot_Mode = SHOOT_MODE_STOP;
		}

		osDelayUntil(TimeIncrement_mode + 2);
		TimeIncrement_mode = osKernelGetTickCount();
	}
}
/**
 *@Function:		Get_Chassis_RC()
 *@Description:	获取底盘遥控器速度
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Chassis_RC()
{
	// 手动限制最大速度
	rc.vx = -DBUS.RC.ch1 / 660.0f * 7000.0f;
	rc.vy = -DBUS.RC.ch0 / 660.0f * 7000.0f;
	rc.vw = DBUS.RC.ch2 / 660.0f * 1500.0f;
}
ramp_t fb_ramp = RAMP_GEN_DAFAULT;
ramp_t lr_ramp = RAMP_GEN_DAFAULT;
ramp_t rotate_ramp = RAMP_GEN_DAFAULT;
ramp_t slow_ramp = RAMP_GEN_DAFAULT;
ramp_t close_ramp = RAMP_GEN_DAFAULT;
ramp_t slow_fb_ramp = RAMP_GEN_DAFAULT;
ramp_t slow_rl_ramp = RAMP_GEN_DAFAULT;
/**
 *@Function:		Get_Chassis_Keyboard()
 *@Description:	获取底盘键盘速度
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Chassis_Keyboard()
{
	static float current_speed_x = 0;
	static float current_speed_y = 0;

	// 前进后退
	if (((DBUS.PC.Keyboard & KEY_W) && !(DBUS.PC.Keyboard & KEY_CTRL)) || ((DBUS.PC.Keyboard & KEY_W) && (DBUS.PC.Keyboard & KEY_CTRL) && (DBUS.PC.Keyboard & KEY_C))) // 防止逃跑时关小陀螺进入慢速模式
	{
		keyboard.vx = current_speed_x + (7000 - current_speed_x) * ramp_calc(&fb_ramp);
		ramp_init(&slow_ramp, 2000); // 2000
	}
	else if (((DBUS.PC.Keyboard & KEY_S) && !(DBUS.PC.Keyboard & KEY_CTRL)) || ((DBUS.PC.Keyboard & KEY_S) && (DBUS.PC.Keyboard & KEY_CTRL) && (DBUS.PC.Keyboard & KEY_C)))
	{
		keyboard.vx = current_speed_x + (-7000 - current_speed_x) * ramp_calc(&fb_ramp);
		ramp_init(&slow_ramp, 2000);
	}
	else if ((DBUS.PC.Keyboard & KEY_W) && (DBUS.PC.Keyboard & KEY_CTRL) && !(DBUS.PC.Keyboard & KEY_C)) // 慢速模式 用于加弹时调整 Ctrl+移动键
	{
		keyboard.vx = 500;
		ramp_init(&fb_ramp, 2000);
	}
	else if ((DBUS.PC.Keyboard & KEY_S) && (DBUS.PC.Keyboard & KEY_CTRL) && !(DBUS.PC.Keyboard & KEY_C))
	{
		keyboard.vx = -500;
		ramp_init(&fb_ramp, 2000);
	}
	else
	{
		keyboard.vx = current_speed_x * (1 - ramp_calc(&slow_ramp));
		ramp_init(&fb_ramp, 2000);
	}

	// 左右平移
	if (((DBUS.PC.Keyboard & KEY_A) && !(DBUS.PC.Keyboard & KEY_CTRL)) || ((DBUS.PC.Keyboard & KEY_A) && (DBUS.PC.Keyboard & KEY_CTRL) && (DBUS.PC.Keyboard & KEY_C)))
	{
		keyboard.vy = current_speed_y + (-7000 - current_speed_y) * ramp_calc(&lr_ramp);
		ramp_init(&slow_ramp, 2000);
	}
	else if (((DBUS.PC.Keyboard & KEY_D) && !(DBUS.PC.Keyboard & KEY_CTRL)) || ((DBUS.PC.Keyboard & KEY_D) && (DBUS.PC.Keyboard & KEY_CTRL) && (DBUS.PC.Keyboard & KEY_C)))
	{
		keyboard.vy = current_speed_y + (7000 - current_speed_y) * ramp_calc(&lr_ramp);
		ramp_init(&slow_ramp, 2000);
	}
	else if ((DBUS.PC.Keyboard & KEY_A) && (DBUS.PC.Keyboard & KEY_CTRL) && !(DBUS.PC.Keyboard & KEY_C))
	{
		keyboard.vy = -500;
		ramp_init(&lr_ramp, 2000);
	}
	else if ((DBUS.PC.Keyboard & KEY_D) && (DBUS.PC.Keyboard & KEY_CTRL) && !(DBUS.PC.Keyboard & KEY_C))
	{
		keyboard.vy = 500;
		ramp_init(&lr_ramp, 2000);
	}
	else
	{
		keyboard.vy = 0;
		ramp_init(&lr_ramp, 2000);
	}
	current_speed_x = keyboard.vx;
	current_speed_y = keyboard.vy;
}

int Cover_Open_Flag = 0;	 // 键盘翻盖控制状态标志位
int Shoot_Run_Flag = 0;		 // 键盘摩擦轮开关状态标志位
int Chassis_Rotate_Flag = 0; // 键盘小陀螺开关状态标志位
/**
 *@Function:		Get_Keyboard_Mode()
 *@Description:	键盘获取模式
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Keyboard_Mode()
{
	// 键盘控制弹仓盖开关
	if (DBUS.PC.Keyboard & KEY_X && DBUS.PC.Keyboard & KEY_CTRL)
	{
		Cover_Open_Flag--;
	}
	else if (DBUS.PC.Keyboard & KEY_X)
	{
		Cover_Open_Flag++;
	}
	else
	{
		Cover_Open_Flag = 0;
	}

	// 键盘控制摩擦轮开关
	if (DBUS.PC.Keyboard & KEY_V && DBUS.PC.Keyboard & KEY_CTRL)
	{
		Shoot_Run_Flag--;
	}
	else if (DBUS.PC.Keyboard & KEY_V)
	{
		// UI重新发送
		Send_Once_Flag = 0;
		Shoot_Run_Flag++;
	}
	else
	{
		Shoot_Run_Flag = 0;
	}

	// 键盘控制底盘模式操作
	if (DBUS.PC.Keyboard & KEY_C && DBUS.PC.Keyboard & KEY_CTRL)
	{
		Chassis_Rotate_Flag--;
	}
	else if (DBUS.PC.Keyboard & KEY_C)
	{
		Chassis_Rotate_Flag++;
	}
	else
	{
		Chassis_Rotate_Flag = 0;
	}
}

// 底盘状态标志位
typedef enum
{
	CHASSIS_STATUS_STOP = 0,
	CHASSIS_STATUS_ROTATE,
	CHASSIS_STATUS_FOLLOW,

} Chassis_Status_Enum;
// 底盘状态（按键用）
Chassis_Status_Enum Chassis_Status = CHASSIS_STATUS_FOLLOW;

/**
 *@Function:		Get_Chassis_Mode()
 *@Description:	获取底盘模式
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Chassis_Mode()
{
	// 双下底盘速度为0
	if (DBUS.RC.Switch_Left == RC_SW_DOWN && DBUS.RC.Switch_Right == RC_SW_DOWN)
	{
		Chassis_Mode = CHASSIS_MODE_STOP;
	}
	else
	{

		// 键盘底盘陀螺开关控制
		if (Chassis_Rotate_Flag > 50) // 50*2 = 100ms
		{
			Chassis_Status = CHASSIS_STATUS_ROTATE;
			Chassis_Rotate_Flag = 0;
		}
		else if (Chassis_Rotate_Flag < -50)
		{
			Chassis_Status = CHASSIS_STATUS_FOLLOW;
			Chassis_Rotate_Flag = 0;
		}

		// 遥控器底盘模式控制
		if (DBUS.RC.Switch_Right == RC_SW_UP || Chassis_Status == CHASSIS_STATUS_FOLLOW)
		{
			Chassis_Mode = CHASSIS_MODE_FOLLOW;
		}

		if (DBUS.RC.Switch_Right == RC_SW_DOWN || Chassis_Status == CHASSIS_STATUS_ROTATE)
		{
			Chassis_Mode = CHASSIS_MODE_ROTATE;
		}

		if (DBUS.RC.Switch_Right == RC_SW_MID)
		{
			Chassis_Mode = CHASSIS_MODE_STOP;
		}
	}
}
/**
 *@Function:		Get_Gimbal_Mode()
 *@Description:	获取云台模式
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Gimbal_Mode()
{
	// 双下无力
	if (DBUS.RC.Switch_Left == RC_SW_DOWN && DBUS.RC.Switch_Right == RC_SW_DOWN)
	{
		Gimbal_Mode = GIMBAL_MODE_STOP;
	}
	else
	{
		// 键盘云台模式获取
		if (DBUS.RC.Switch_Right == RC_SW_UP)
		{
			Gimbal_Mode = GIMBAL_MODE_FOLLOW;
		}

		// 右键按住自瞄，或者右拨杆置中
		if (DBUS.RC.Switch_Right == RC_SW_MID || DBUS.PC.Press_Right)
		{
			Gimbal_Mode = GIMBAL_MODE_AUTOAIM;
		}

		// 右下底盘模式为旋转，此时云台保持跟随模式
		else if (DBUS.RC.Switch_Right == RC_SW_DOWN)
		{
			Gimbal_Mode = GIMBAL_MODE_FOLLOW;
		}
	}
}

// 弹仓盖翻转标志位
int Cover_Ready_Flag = 0; // 遥控器
int Cover_Now_Flag = 0;	  // 0代表闭合，1代表开启

/**
 *@Function:		Get_Cover_Mode()
 *@Description:	获取翻盖模式
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Cover_Mode()
{
	// 翻盖开关控制
	if (Cover_Open_Flag > 50) // 50*2 = 100ms
	{
		COVER_OFF;
		Cover_Open_Flag = 0;
		Cover_Now_Flag = 0;
	}
	else if (Cover_Open_Flag < -50)
	{
		COVER_ON;
		Cover_Open_Flag = 0;
		Cover_Now_Flag = 1;
	}

	// 拨轮控制弹仓盖，往下拨，回正到一定位置后变更一次状态
	if (DBUS.RC.ch4 > 600)
	{
		Cover_Ready_Flag = 1;
	}
	if (Cover_Ready_Flag == 1)
	{
		// 回正到接近中间位置
		if (DBUS.RC.ch4 < 100 && DBUS.RC.ch4 > 0)
		{

			if (Cover_Now_Flag == 0)
			{
				COVER_ON;
				Cover_Now_Flag = 1;
			}
			else if (Cover_Now_Flag == 1)
			{
				COVER_OFF;
				Cover_Now_Flag = 0;
			}
			Cover_Ready_Flag = 0;
		}
	}
}

int Switch_Left_Last = RC_SW_UP; // 左拨杆上一状态
int Shoot_Angle_Bullet = 36856;
int heat_control_flag1 = 1; // 标志位
int heat_control_flag2 = 1; // 标志位
int shoot_speed = 0;		// 射速
int last_shoot_speed = 0;	// 上次的射速
/**
 *@Function:		Get_Shoot_Mode()
 *@Description:	获取发射模式
 *@Param:       形参
 *@Return:	  	返回值
 */
void Get_Shoot_Mode()
{
	// 双下无力
	if (DBUS.RC.Switch_Left == RC_SW_DOWN && DBUS.RC.Switch_Right == RC_SW_DOWN)
	{
		Shoot_Mode = SHOOT_MODE_STOP;
	}
	else
	{
		// 发射开关控制
		if (Shoot_Run_Flag > 50) // 50*2 = 100ms
		{
			Shoot_Mode = SHOOT_MODE_RUN;
			Shoot_Run_Flag = 0;
		}
		else if (Shoot_Run_Flag < -50)
		{
			Shoot_Mode = SHOOT_MODE_STOP;
			Shoot_Run_Flag = 0;
		}

		// 遥控器控制摩擦轮开关
		if (Switch_Left_Last == RC_SW_MID)
		{
			if (DBUS.RC.Switch_Left == RC_SW_UP)
			{
				if (Shoot_Mode == SHOOT_MODE_RUN)
				{
					Shoot_Mode = SHOOT_MODE_STOP;
				}
				else if (Shoot_Mode == SHOOT_MODE_STOP)
				{
					Shoot_Mode = SHOOT_MODE_RUN;
				}
			}
		}
		Switch_Left_Last = DBUS.RC.Switch_Left;

		// 仅在开启摩擦轮的情况下有效
		if (Shoot_Mode == SHOOT_MODE_RUN)
		{
			// 遥控器控制单发连发
			if (DBUS.RC.Switch_Left == RC_SW_DOWN || DBUS.PC.Press_Left)
			{
				Trig_Time++;
			}
			else
			{
				Trig_Time = 0;
			}
			// 短按2ms
			if (Trig_Time == 1)
			{
				Shoot_Cmd = SHOOT_CMD_ONCE;
				Shoot_Aim_Angle = CAN_Trigger.Total_MechAngle + Shoot_Angle_Bullet;

				// 单发标志
				Shoot_Once_Flag = 1;
			}
			// 长按200ms
			else if (Trig_Time > 100)
			{
				Shoot_Cmd = SHOOT_CMD_LONG;
			}
		}
	}

	// 剩余热量控制
	{
		if ((DBUS.PC.Keyboard & KEY_R) && !(DBUS.PC.Keyboard & KEY_CTRL)) // 按R加一颗弹丸的热量剩余
		{
			if (heat_control_flag1 == 1)
			{
				heat_control++;
			}
			heat_control_flag1 = 0;
		}
		else
		{
			heat_control_flag1 = 1;
		}

		if ((DBUS.PC.Keyboard & KEY_R) && (DBUS.PC.Keyboard & KEY_CTRL)) // 按ctrl+G减一颗弹丸的热量剩余
		{
			if (heat_control_flag2 == 1)
			{
				heat_control--;
			}
			heat_control_flag2 = 0;
		}
		else
		{
			heat_control_flag2 = 1;
		}
	}

	// 子弹计数
	{
		shoot_speed = Referee_Inf.real_shoot_data.bullet_speed;
		if (shoot_speed != last_shoot_speed) // 射速变化表明打了一个弹
		{
			shoot_counter++;
			last_shoot_speed = shoot_speed;
		}
	}
}
