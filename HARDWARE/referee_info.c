#include "referee_info.h"



/*
 *@title：float8位转32位
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
float bit8TObit32(uint8_t *change_info)
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
/*
 *@title：int32位转8位
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
char bit32TObit8(int index_need, int bit32)
{
	union
	{
		float f;
		char byte[4];
	} u32val;
	u32val.f = bit32;
	return u32val.byte[index_need];
}
/*
 *@title：8位转16位
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
int16_t bit8TObit16(uint8_t *change_info)
{
	union
	{
		int16_t f;
		char byte[2];
	} u16val;
	u16val.byte[0] = change_info[0];
	u16val.byte[1] = change_info[1];
	return u16val.f;
}
/*
 *@title：int16位转8位
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
char bit16TObit8(int index_need, int bit16)
{
	union
	{
		float f;
		char byte[2];
	} u16val;
	u16val.f = bit16;
	return u16val.byte[index_need];
}
int robot_hp_decrease_vel = 0;
int robot_hp_last = 0;
int robot_time = 0;
int robot_time_last = 0;
int robot_delay = 0;
u16 length;

int armor_detect_flag = 0;
/**
 *@Function:		RefereeInfo_Decode(uint8_t *Referee_Info)
 *@Description:	裁判系统数据解析
 *@Param:       形参
 *@Return:	  	返回值
 */
void RefereeInfo_Decode(uint8_t *Referee_Info)
{
	// 哨兵默认6
	Referee_Inf.robot_hurt_data.armor_id = 6;
	Referee_Inf.robot_hurt_data.hurt_type = 6;
	// CMD命令码
	int cmd = bit8TObit16(&Referee_Info[5]);
	// 根据命令码接收数据
	switch (cmd)
	{
	case 0x0003:
	{
		Referee_Inf.robot_hp.blue_outpost_HP = bit8TObit16(&Referee_Info[7 + 28]);
		Referee_Inf.robot_hp.red_outpost_HP = bit8TObit16(&Referee_Info[7 + 12]);
		Referee_Inf.robot_hp.blue_7_robot_HP = bit8TObit16(&Referee_Info[7 + 26]);
		Referee_Inf.robot_hp.red_7_robot_HP = bit8TObit16(&Referee_Info[7 + 10]);
	}
	break;

	case 0x0201:
	{
		Referee_Inf.game_robot_state.robot_id = Referee_Info[7];
		Referee_Inf.game_robot_state.robot_level = Referee_Info[8];
		Referee_Inf.game_robot_state.remain_HP = bit8TObit16(&Referee_Info[9]);
		Referee_Inf.game_robot_state.max_HP = bit8TObit16(&Referee_Info[11]);
		Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_rate = bit8TObit16(&Referee_Info[13]);
		Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_limit = bit8TObit16(&Referee_Info[15]);
		Referee_Inf.game_robot_state.shooter_id1_17mm_speed_limit = bit8TObit16(&Referee_Info[17]);
		Referee_Inf.game_robot_state.shooter_id2_17mm_cooling_rate = bit8TObit16(&Referee_Info[19]);
		Referee_Inf.game_robot_state.shooter_id2_17mm_cooling_limit = bit8TObit16(&Referee_Info[21]);
		Referee_Inf.game_robot_state.shooter_id2_17mm_speed_limit = bit8TObit16(&Referee_Info[23]);
		Referee_Inf.game_robot_state.shooter_id1_42mm_cooling_rate = bit8TObit16(&Referee_Info[25]);
		Referee_Inf.game_robot_state.shooter_id1_42mm_cooling_limit = bit8TObit16(&Referee_Info[27]);
		Referee_Inf.game_robot_state.shooter_id1_42mm_speed_limit = bit8TObit16(&Referee_Info[29]);
		Referee_Inf.game_robot_state.chassis_power_limit = bit8TObit16(&Referee_Info[31]);

		// 红蓝方ID获取
		// 蓝方
		if (Referee_Inf.game_robot_state.robot_id >= 101)
		{
			Referee_Inf.Robot_Color = BLUE;
		}
		// 红方
		else
		{
			Referee_Inf.Robot_Color = RED;
		}
	}
	break;
	case 0x0202:
	{
		Referee_Inf.power_heat_data.chassis_volt = bit8TObit16(&Referee_Info[7]);
		Referee_Inf.power_heat_data.chassis_current = bit8TObit16(&Referee_Info[9]);
		Referee_Inf.power_heat_data.chassis_power = bit8TObit32(&Referee_Info[11]);		   // 底盘功率
		Referee_Inf.power_heat_data.chassis_power_buffer = bit8TObit16(&Referee_Info[15]); // 底盘功率缓冲
		Referee_Inf.power_heat_data.shooter_id1_17mm_cooling_heat = bit8TObit16(&Referee_Info[17]);
		Referee_Inf.power_heat_data.shooter_id2_17mm_cooling_heat = bit8TObit16(&Referee_Info[19]);
		Referee_Inf.power_heat_data.shooter_id1_42mm_cooling_heat = bit8TObit16(&Referee_Info[21]); // 枪口热量
	}
	break;
	case 0x0204:
	{
		Referee_Inf.buff_data.power_rune_buff = Referee_Info[7];
	}
	break;
	case 0x0206:
	{
		Referee_Inf.robot_hurt_data.armor_id = Referee_Info[7] & 0x0f;
		Referee_Inf.robot_hurt_data.hurt_type = Referee_Info[7] & 0xf0;

		if (Referee_Inf.robot_hurt_data.armor_id == 0 && Referee_Inf.robot_hurt_data.hurt_type == 0x0)
		{
			// armor_detect_flag ++;
		}
	}
	break;
	case 0x0207:
	{
		Referee_Inf.real_shoot_data.shooter_id = Referee_Info[8];
		Referee_Inf.real_shoot_data.bullet_freq = Referee_Info[9];
		Referee_Inf.real_shoot_data.bullet_speed = bit8TObit32(&Referee_Info[10]);
	}
	break;
	case 0x0208:
	{
		Referee_Inf.bullet_remaining.bullet_remaining_num_17mm = bit8TObit16(&Referee_Info[7]);
	}
	break;

	// 客户端下发信息
	// 云台手控制
	case 0x0303:
	{
		memcpy(&Referee_Inf.robot_command_data, &Referee_Info[7], Referee_Info[2] << 8 | Referee_Info[1]);
	}
	break;

	default:
	{
	}
	break;
	}
}
