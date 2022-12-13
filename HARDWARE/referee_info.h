#ifndef __REFEREE_INFO_H
#define __REFEREE_INFO_H

#include "main.h"

#define BLUE 1
#define RED 0

#define MAX_SIZE 200	  // 上传数据寄存器最大的长度
#define frameheader_len 5 // 帧头长度
#define cmd_len 2		  // 命令码长度
#define crc_len 2		  // CRC16校验

// 帧头格式
// typedef struct
// {					// 字节偏移     大小     说明
// 	u8 SOF;			//   0          1      数据帧起始字节，固定值为0XA5
// 	u16 DataLength; //   1          2      数据帧内Data长度
// 	u8 Seq;			//   3          1      包序号
// 	u8 CRC8;		//   4          1      帧头CRC校验
// } FrameHeader_TypeDef;

// // 通信协议格式
// typedef struct
// {									 // 字节偏移     大小     说明
// 	FrameHeader_TypeDef FrameHeader; //   0          5      帧头协议
// 	u16 CmdID;						 //   5          2      命令码ID
// 	u16 Date[50];					 //   7          n      数据
// 	u16 FrameTail;					 //   7+n        2      CRC16校验
// } RefereeInfo_TypeDef;

#define GAME_STATE_ID = 0x0001	  // 比赛状态数据
#define GAME_RESULT_ID = 0x0002	  // 比赛结果数据
#define ROBO_SURVIVOR_ID = 0x0003 // 比赛机器人血量数据
#define DART_STATUS_ID = 0x0004	  // 飞镖发射状态

#define EVENE_DATA_ID = 0x0101			// 场地事件数据
#define SUPPLY_ACTION_ID = 0x0102		// 场地补给站动作标识数据
#define SUPPLY_BOOKING_ID = 0x0103		// 请求补给站补弹数据
#define REFEREE_WARNING_ID = 0x0104		// 裁判警告数据
#define DART_REMAINING_TIME_ID = 0x0105 // 飞镖发射口倒计时

#define GAME_ROBO_STATE_ID = 0x0201	   // 机器人状态数据
#define REAL_POWER_HEAT_ID = 0x0202	   // 实时功率热量数据
#define ROBOT_POSITION_ID = 0x0203	   // 机器人位置数据
#define BUFF_MUSK_ID = 0x0204		   // 机器人增益数据
#define AERIAL_ROBO_ENERGY_ID = 0x0205 // 空中机器人能量状态数据
#define ROBOT_HURT_ID = 0x0206		   // 伤害状态数据
#define REAL_SHOOT_DATA_ID = 0x0207	   // 实时射击数据
#define BULLET_REMAINING_ID = 0x0208   // 子弹剩余发送数，空中机器人以及哨兵机器人发送
#define DFID_STATUS_ID = 0x0209		   // 机器人 RFID 状态

#define INTERACTIVE_DATA_ID = 0x0301





void RefereeInfo_Decode(uint8_t *Referee_Info);

#endif
