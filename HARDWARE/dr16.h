#ifndef __DR16_H
#define __DR16_H
#include "main.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W ((uint16_t)0x01 << 0)		// 1
#define KEY_S ((uint16_t)0x01 << 1)		// 2
#define KEY_A ((uint16_t)0x01 << 2)		// 4
#define KEY_D ((uint16_t)0x01 << 3)		// 8
#define KEY_SHIFT ((uint16_t)0x01 << 4) // 16
#define KEY_CTRL ((uint16_t)0x01 << 5)	// 32
#define KEY_Q ((uint16_t)0x01 << 6)		// 64
#define KEY_E ((uint16_t)0x01 << 7)		// 128
#define KEY_R ((uint16_t)0x01 << 8)		// 256
#define KEY_F ((uint16_t)0x01 << 9)		// 512
#define KEY_G ((uint16_t)0x01 << 10)	// 1024
#define KEY_Z ((uint16_t)0x01 << 11)	// 2048
#define KEY_X ((uint16_t)0x01 << 12)	// 4096
#define KEY_C ((uint16_t)0x01 << 13)	// 8192
#define KEY_V ((uint16_t)0x01 << 14)	// 16384
#define KEY_B ((uint16_t)0x01 << 15)	// 32768

/* ----------------------- Data Struct ------------------------------------- */

void DR16_DataDecode(volatile const uint8_t *dbus_rx_buff);
void DR16_Init();
extern void USAR_UART3_IDLECallback(UART_HandleTypeDef *huart);

#endif