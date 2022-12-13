#include "can_h.h"

volatile static unsigned char CAN1_Tx_Message_Flag = 0; // CAN1发送消息标志
volatile static unsigned char CAN2_Tx_Message_Flag = 0; // CAN2发送消息标志

void CAN_Data_Decode(CAN_Data_TypeDef *CAN_Data, uint8_t Rx_Data[])
{
	CAN_Data->Origin_MechAngle = (Rx_Data[0] << 8) | (Rx_Data[1]);
	CAN_Data->Current_Speed = (Rx_Data[2] << 8) | (Rx_Data[3]);
	CAN_Data->Current = (Rx_Data[4] << 8) | (Rx_Data[5]);
	CAN_Data->Temperature = (Rx_Data[6]);

	if (CAN_Data->Critical_MechAngle != 0) // 当前数据为云台电机使用，处理角度数据并转换角度范围
	{
		if (CAN_Data->Origin_MechAngle < CAN_Data->Critical_MechAngle) // 3000
		{
			CAN_Data->Current_MechAngle = CAN_Data->Origin_MechAngle + 8192;
		}
		else
		{
			CAN_Data->Current_MechAngle = CAN_Data->Origin_MechAngle;
		}
	}
	else // 普通电机使用原始的角度值
	{
		CAN_Data->Current_MechAngle = CAN_Data->Origin_MechAngle;
	}
}

// 圈数记录
void CAN_Total_Angle_Decode(CAN_Data_TypeDef *CAN_Data)
{
	static int Init_Flag = 0;
	static int Total_Num = 0;
	static int Init_Angle = 0;

	if (!Init_Flag)
	{
		Init_Angle = CAN_Data->Current_MechAngle; // 消除初始角度的误差
		Init_Flag = 1;
	}
	// 记圈数
	if ((CAN_Data->Current_MechAngle - CAN_Data->Last_MechAngle) > 4095)
	{
		Total_Num -= 1;
	}
	else if ((CAN_Data->Current_MechAngle - CAN_Data->Last_MechAngle) < -4095)
	{
		Total_Num += 1;
	}

	CAN_Data->Total_MechAngle = Total_Num * 8192 + (CAN_Data->Current_MechAngle) - Init_Angle; // 第一次拨弹肯定是正转所以只用减去初始角
	CAN_Data->Last_MechAngle = CAN_Data->Current_MechAngle;
}

static float bit8TObit32(uint8_t *change_info)
{
	union
	{
		float f;
		char byte[4];
	} u32val;
	u32val.byte[0] = change_info[0];
	u32val.byte[1] = change_info[1];
	u32val.byte[2] = change_info[2];
	u32val.byte[3] = change_info[3];
	return u32val.f;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if (hcan == &hcan1)
	{
		CAN_RxHeaderTypeDef CAN1_Rx_Message;
		uint8_t CAN1_Rx_Data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_Rx_Message, CAN1_Rx_Data);

		if ((CAN1_Rx_Message.IDE == CAN_ID_STD) && (CAN1_Rx_Message.RTR == CAN_RTR_DATA) && (CAN1_Rx_Message.DLC == 8))
		{
			CAN1_Signal = 100;
			switch (CAN1_Rx_Message.StdId)
			{
			// 底盘
			case 0x201:
			case 0x202:
			case 0x203:
			case 0x204:
			{
				CAN_Data_Decode(&CAN_Chassis[CAN1_Rx_Message.StdId - 0x201], CAN1_Rx_Data);
			}
			break;

			// Yaw
			case 0x205:
			{
				CAN_Data_Decode(&CAN_Gimbal[0], CAN1_Rx_Data);
			}
			break;

			// Pitch
			case 0x209:
			{
				CAN_Data_Decode(&CAN_Gimbal[1], CAN1_Rx_Data);
			}

			// SuperCap(雾列电容)
			case 0x211:
			{
				// 输入电压
				// 电容电压
				// 输入电流
				// 设定功率
				uint16_t *pPowerData = (uint16_t *)CAN1_Rx_Data;
				SuperCap_Info.InputVot = (float)pPowerData[0] / 100.0f;
				SuperCap_Info.CapVot = (float)pPowerData[1] / 100.0f;
				SuperCap_Info.InputCurrent = (float)pPowerData[2] / 100.0f;
				SuperCap_Info.CurrentPower = (float)pPowerData[3] / 100.0f;
				SuperCap_Info.Low_Filter_Vot = 0.3f * SuperCap_Info.CapVot + (1.0f - 3.0f) * SuperCap_Info.Low_Filter_Vot;
				SuperCap_Info.id = 0x211;

				SuperCap_Signal = 1000;
			}
			break;

			// SuperCap(自制)
			case 0x300:
			{
				// 输入电压
				// 输入电流
				SuperCap_Info.CapVot = bit8TObit32(CAN1_Rx_Data);
				SuperCap_Info.InputCurrent = bit8TObit32(&CAN1_Rx_Data[4]);
				SuperCap_Info.id = 0x300;

				SuperCap_Signal = 1000;
			}
			break;

			default:
			{
				break;
			}
			}
		}
	}
	else if (hcan == &hcan2)
	{ // can2
		CAN_RxHeaderTypeDef CAN2_Rx_Message;
		uint8_t CAN2_Rx_Data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_Rx_Message, CAN2_Rx_Data);
		if ((CAN2_Rx_Message.IDE == CAN_ID_STD) && (CAN2_Rx_Message.RTR == CAN_RTR_DATA) && (CAN2_Rx_Message.DLC == 8))
		{
			CAN2_Signal = 100;
			switch (CAN2_Rx_Message.StdId)
			{

			// 拨单盘电机
			case 0x202:
			{
				CAN_Data_Decode(&CAN_Trigger, CAN2_Rx_Data);
				CAN_Total_Angle_Decode(&CAN_Trigger);
			}
			break;

			// 左摩擦轮
			case 0x203:
			{
				CAN_Data_Decode(&CAN_Shoot[0], CAN2_Rx_Data);
			}
			break;

			// 右摩擦轮
			case 0x204:
			{
				CAN_Data_Decode(&CAN_Shoot[1], CAN2_Rx_Data);
			}
			break;
				//
				//
				//								case 0x205:
				//								case 0x206:
				//								case 0x207:
				//								case 0x208:
				//								{
				//
				//								}break;
				//
			default:
			{
				break;
			}
			}
		}
	}
}

/**
 *@Function:		CAN1_TX_Chassis(void)
 *@Description:	CAN1底盘数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
static int now_count = 0;
void CAN1_TX_Chassis(void)
{
	static int last_count = 0;
	static int cur_count = 0;
	CAN_TxHeaderTypeDef CAN1_Tx_Message;
	uint8_t CAN1_Tx_Data[8];
	uint32_t send_mail_box;

	CAN1_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
	CAN1_Tx_Message.DLC = 0x08;			// 帧长度为8
	CAN1_Tx_Message.StdId = 0x200;		// 帧ID为传入参数的CAN_ID

	CAN1_Tx_Data[0] = (CAN_Chassis[0].Target_Current >> 8) & 0xff; // 201接收电流高8位
	CAN1_Tx_Data[1] = (CAN_Chassis[0].Target_Current) & 0xff;	   // 201接收电流低8位
	CAN1_Tx_Data[2] = (CAN_Chassis[1].Target_Current >> 8) & 0xff; // 202接收电流高8位
	CAN1_Tx_Data[3] = (CAN_Chassis[1].Target_Current) & 0xff;	   // 202接收电流低8位
	CAN1_Tx_Data[4] = (CAN_Chassis[2].Target_Current >> 8) & 0xff; // 203接收电流高8位
	CAN1_Tx_Data[5] = (CAN_Chassis[2].Target_Current) & 0xff;	   // 203接收电流低8位
	CAN1_Tx_Data[6] = (CAN_Chassis[3].Target_Current >> 8) & 0xff; // 204接收电流高8位
	CAN1_Tx_Data[7] = (CAN_Chassis[3].Target_Current) & 0xff;	   // 204接收电流低8位
#if CHASSIS_OFF
	CAN1_Tx_Data[0] = 0; // 201接收电流高8位
	CAN1_Tx_Data[1] = 0; // 201接收电流低8位
	CAN1_Tx_Data[2] = 0; // 202接收电流高8位
	CAN1_Tx_Data[3] = 0; // 202接收电流低8位
	CAN1_Tx_Data[4] = 0; // 203接收电流高8位
	CAN1_Tx_Data[5] = 0; // 203接收电流低8位
	CAN1_Tx_Data[6] = 0; // 204接收电流高8位
	CAN1_Tx_Data[7] = 0; // 204接收电流低8位
#endif
	CAN1_Tx_Message_Flag = 0;

	// CAN1掉线处理
	if (CAN1_Signal > 0)
	{
		HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box); // 传输
	}
	int count = 0;
	while (CAN1_Tx_Message_Flag == 0)
	{
		count++;

		if (count > 50)
		{
			HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
			count = 0;
		}

		if (CAN1_Signal <= 0)
		{
			break;
		}
	}

	now_count = HAL_GetTick() - last_count;
	last_count = HAL_GetTick();
}
/**
 *@Function:		CAN2_TX_DUAL(void)
 *@Description:	CAN2双头通讯
 *@Param:       形参
 *@Return:	  	返回值
 */
void CAN2_TX_DUAL(void)
{
	CAN_TxHeaderTypeDef CAN2_Tx_Message;

	uint8_t CAN2_Tx_Data[8];
	uint32_t send_mail_box;

	CAN2_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
	CAN2_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
	CAN2_Tx_Message.DLC = 0x08;			// 帧长度为8
	CAN2_Tx_Message.StdId = 0x102;		// 帧ID为传入参数的CAN_ID

	CAN2_Tx_Data[0] = (DBUS.RC.ch2 >> 8) & 0xff;
	CAN2_Tx_Data[1] = (DBUS.RC.ch2) & 0xff;
	CAN2_Tx_Data[2] = (DBUS.RC.ch3 >> 8) & 0xff;
	CAN2_Tx_Data[3] = (DBUS.RC.ch3) & 0xff;
	CAN2_Tx_Data[4] = (DBUS.RC.ch4 >> 8) & 0xff;
	CAN2_Tx_Data[5] = (DBUS.RC.ch4) & 0xff;
	CAN2_Tx_Data[6] = DBUS.RC.Switch_Left;
	CAN2_Tx_Data[7] = DBUS.RC.Switch_Right;

	CAN2_Tx_Message_Flag = 0;
	// 防止一个头出问题另一个头寄掉
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Tx_Message, CAN2_Tx_Data, &send_mail_box);

	while (CAN2_Tx_Message_Flag == 0)
	{
		HAL_CAN_AddTxMessage(&hcan2, &CAN2_Tx_Message, CAN2_Tx_Data, &send_mail_box);
		if (CAN2_Signal <= 0)
		{
			break;
		}
	}
}
/**
 *@Function:		CAN1_TX_Gimbal_Pitch(void)
 *@Description:	CAN1云台Pitch数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
static int pitch_count = 0;
void CAN1_TX_Gimbal_Pitch(void)
{
	static int last_count = 0;
	static int num = 0;
	CAN_TxHeaderTypeDef CAN1_Tx_Message;
	uint8_t CAN1_Tx_Data[8];
	uint32_t send_mail_box;

	CAN1_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
	CAN1_Tx_Message.DLC = 0x08;			// 帧长度为8
	CAN1_Tx_Message.StdId = 0x2FF;		// 帧ID为传入参数的CAN_ID

	CAN1_Tx_Data[0] = (CAN_Gimbal[1].Target_Current >> 8) & 0xff; // 201接收电流高8位
	CAN1_Tx_Data[1] = (CAN_Gimbal[1].Target_Current) & 0xff;	  // 201接收电流低8位
#if PITCH_OFF
	CAN1_Tx_Data[0] = 0; // 201接收电流高8位
	CAN1_Tx_Data[1] = 0; // 201接收电流低8位
#endif
	CAN1_Tx_Data[2] = 0;
	CAN1_Tx_Data[3] = 0;
	CAN1_Tx_Data[4] = 0;
	CAN1_Tx_Data[5] = 0;
	CAN1_Tx_Data[6] = 0;
	CAN1_Tx_Data[7] = 0;

	CAN1_Tx_Message_Flag = 0;

	// CAN1掉线处理
	if (CAN1_Signal > 0)
	{

		HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
	}
	int count = 0;
	while (CAN1_Tx_Message_Flag == 0)
	{
		count++;

		if (count >= 500)
		{

			HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
			count = 0;
		}
		if (CAN1_Signal <= 0)
		{
			break;
		}
	}

	pitch_count = HAL_GetTick() - last_count;
	if (pitch_count > 10)
		num++;
	last_count = HAL_GetTick();
}

/**
 *@Function:		CAN1_TX_Gimbal_Yaw(void)
 *@Description:	CAN1云台Yaw数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
static int yaw_count = 0;
void CAN1_TX_Gimbal_Yaw(void)
{
	static int last_count = 0;
	CAN_TxHeaderTypeDef CAN1_Tx_Message;
	uint8_t CAN1_Tx_Data[8];
	uint32_t send_mail_box;

	CAN1_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
	CAN1_Tx_Message.DLC = 0x08;			// 帧长度为8
	CAN1_Tx_Message.StdId = 0x1FF;		// 帧ID为传入参数的CAN_ID

	CAN1_Tx_Data[0] = (CAN_Gimbal[0].Target_Current >> 8) & 0xff; // 201接收电流高8位
	CAN1_Tx_Data[1] = (CAN_Gimbal[0].Target_Current) & 0xff;	  // 201接收电流低8位
#if YAW_OFF
	CAN1_Tx_Data[0] = 0; // 201接收电流高8位
	CAN1_Tx_Data[1] = 0; // 201接收电流低8位
#endif
	CAN1_Tx_Data[2] = 0;
	CAN1_Tx_Data[3] = 0;
	CAN1_Tx_Data[4] = 0;
	CAN1_Tx_Data[5] = 0;
	CAN1_Tx_Data[6] = 0;
	CAN1_Tx_Data[7] = 0;

	CAN1_Tx_Message_Flag = 0;

	// CAN1掉线处理
	if (CAN1_Signal > 0)
	{
		HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
	}

	while (CAN1_Tx_Message_Flag == 0)
	{
		HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
		if (CAN1_Signal <= 0)
		{
			break;
		}
	}
	yaw_count = HAL_GetTick() - last_count;

	last_count = HAL_GetTick();
}

/**
 *@Function:		CAN2_TX_Shoot(void)
 *@Description:	CAN2发射数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
void CAN2_TX_Shoot(void)
{
	CAN_TxHeaderTypeDef CAN2_Tx_Message;
	uint8_t CAN2_Tx_Data[8];
	uint32_t send_mail_box;

	CAN2_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
	CAN2_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
	CAN2_Tx_Message.DLC = 0x08;			// 帧长度为8
	CAN2_Tx_Message.StdId = 0x200;		// 帧ID为传入参数的CAN_ID

	CAN2_Tx_Data[0] = 0;
	CAN2_Tx_Data[1] = 0;
	CAN2_Tx_Data[2] = (CAN_Trigger.Target_Current >> 8) & 0xff;
	CAN2_Tx_Data[3] = (CAN_Trigger.Target_Current) & 0xff;
	CAN2_Tx_Data[4] = (CAN_Shoot[0].Target_Current >> 8) & 0xff;
	CAN2_Tx_Data[5] = (CAN_Shoot[0].Target_Current) & 0xff;
	CAN2_Tx_Data[6] = (CAN_Shoot[1].Target_Current >> 8) & 0xff;
	CAN2_Tx_Data[7] = (CAN_Shoot[1].Target_Current) & 0xff;

	CAN2_Tx_Message_Flag = 0;

	// CAN2掉线处理
	if (CAN2_Signal > 0)
	{
		HAL_CAN_AddTxMessage(&hcan2, &CAN2_Tx_Message, CAN2_Tx_Data, &send_mail_box);
	}

	while (CAN2_Tx_Message_Flag == 0)
	{
		HAL_CAN_AddTxMessage(&hcan2, &CAN2_Tx_Message, CAN2_Tx_Data, &send_mail_box);
		if (CAN2_Signal <= 0)
		{
			break;
		}
	}
}

/**
 *@Function:		CAN1_TX_SuperCap(void)
 *@Description:	CAN1超级电容数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
void CAN1_TX_SuperCap(void)
{
	CAN_TxHeaderTypeDef CAN1_Tx_Message;
	uint8_t CAN1_Tx_Data[8];
	uint32_t send_mail_box;

	if (SuperCap_Info.id == 0x211)
	{
		// SuperCap（雾列控制）
		CAN1_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
		CAN1_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
		CAN1_Tx_Message.DLC = 0x08;			// 帧长度为8
		CAN1_Tx_Message.StdId = 0x210;		// 帧ID为传入参数的CAN_ID

		CAN1_Tx_Data[0] = ((Referee_Inf.game_robot_state.chassis_power_limit * 100) >> 8) & 0xff;
		CAN1_Tx_Data[1] = (Referee_Inf.game_robot_state.chassis_power_limit * 100) & 0xff;
		CAN1_Tx_Data[2] = 0;
		CAN1_Tx_Data[3] = 0;
		CAN1_Tx_Data[4] = 0;
		CAN1_Tx_Data[5] = 0;
		CAN1_Tx_Data[6] = 0;
		CAN1_Tx_Data[7] = 0;
	}
	else
	{
		// 根据底盘功率设定系数
		switch (Referee_Inf.game_robot_state.chassis_power_limit)
		{
		case 45:
			power_flag = 1;
			break;
		case 50:
			power_flag = 2;
			break;
		case 55:
			power_flag = 3;
			break;
		case 60:
			power_flag = 4;
			break;
		case 80:
			power_flag = 5;
			break;
		case 100:
			power_flag = 6;
			break;
		default:
			power_flag = 1;
			break;
		}

		// SuperCap（自制）
		CAN1_Tx_Message.IDE = CAN_ID_STD;	// 标准帧
		CAN1_Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
		CAN1_Tx_Message.DLC = 0x08;			// 帧长度为8
		CAN1_Tx_Message.StdId = 0x427;

		CAN1_Tx_Data[0] = power_relay;
		CAN1_Tx_Data[1] = power_flag;
		CAN1_Tx_Data[2] = Referee_Inf.game_robot_state.chassis_power_limit - Referee_Inf.power_heat_data.chassis_power;
		CAN1_Tx_Data[3] = 0;
		CAN1_Tx_Data[4] = 0;
		CAN1_Tx_Data[5] = 0;
		CAN1_Tx_Data[6] = 0;
		CAN1_Tx_Data[7] = 0;
	}

	CAN1_Tx_Message_Flag = 0;

	// CAN1掉线处理
	if (CAN1_Signal > 0)
	{
		HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
	}
	int count_time = 0;
	while (CAN1_Tx_Message_Flag == 0)
	{
		if (CAN1_Signal <= 0)
		{
			break;
		}

		count_time++;
		if (count_time % 100 == 0)
		{
			HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, &send_mail_box);
		}
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		CAN1_Tx_Message_Flag = 1;
	}
	else if (hcan == &hcan2)
	{
		CAN2_Tx_Message_Flag = 1;
	}
}
