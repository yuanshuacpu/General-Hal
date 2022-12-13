#ifndef __REFEREE_INFO_H
#define __REFEREE_INFO_H

#include "main.h"

#define BLUE 1
#define RED 0

#define MAX_SIZE 200	  // �ϴ����ݼĴ������ĳ���
#define frameheader_len 5 // ֡ͷ����
#define cmd_len 2		  // �����볤��
#define crc_len 2		  // CRC16У��

// ֡ͷ��ʽ
// typedef struct
// {					// �ֽ�ƫ��     ��С     ˵��
// 	u8 SOF;			//   0          1      ����֡��ʼ�ֽڣ��̶�ֵΪ0XA5
// 	u16 DataLength; //   1          2      ����֡��Data����
// 	u8 Seq;			//   3          1      �����
// 	u8 CRC8;		//   4          1      ֡ͷCRCУ��
// } FrameHeader_TypeDef;

// // ͨ��Э���ʽ
// typedef struct
// {									 // �ֽ�ƫ��     ��С     ˵��
// 	FrameHeader_TypeDef FrameHeader; //   0          5      ֡ͷЭ��
// 	u16 CmdID;						 //   5          2      ������ID
// 	u16 Date[50];					 //   7          n      ����
// 	u16 FrameTail;					 //   7+n        2      CRC16У��
// } RefereeInfo_TypeDef;

#define GAME_STATE_ID = 0x0001	  // ����״̬����
#define GAME_RESULT_ID = 0x0002	  // �����������
#define ROBO_SURVIVOR_ID = 0x0003 // ����������Ѫ������
#define DART_STATUS_ID = 0x0004	  // ���ڷ���״̬

#define EVENE_DATA_ID = 0x0101			// �����¼�����
#define SUPPLY_ACTION_ID = 0x0102		// ���ز���վ������ʶ����
#define SUPPLY_BOOKING_ID = 0x0103		// ���󲹸�վ��������
#define REFEREE_WARNING_ID = 0x0104		// ���о�������
#define DART_REMAINING_TIME_ID = 0x0105 // ���ڷ���ڵ���ʱ

#define GAME_ROBO_STATE_ID = 0x0201	   // ������״̬����
#define REAL_POWER_HEAT_ID = 0x0202	   // ʵʱ������������
#define ROBOT_POSITION_ID = 0x0203	   // ������λ������
#define BUFF_MUSK_ID = 0x0204		   // ��������������
#define AERIAL_ROBO_ENERGY_ID = 0x0205 // ���л���������״̬����
#define ROBOT_HURT_ID = 0x0206		   // �˺�״̬����
#define REAL_SHOOT_DATA_ID = 0x0207	   // ʵʱ�������
#define BULLET_REMAINING_ID = 0x0208   // �ӵ�ʣ�෢���������л������Լ��ڱ������˷���
#define DFID_STATUS_ID = 0x0209		   // ������ RFID ״̬

#define INTERACTIVE_DATA_ID = 0x0301





void RefereeInfo_Decode(uint8_t *Referee_Info);

#endif
