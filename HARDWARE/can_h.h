#ifndef _CAN_HANDLE
#define _CAN_HANDLE
#include "main.h"

typedef struct CAN_Data_TypeDef CAN_Data_TypeDef;

void CAN1_TX_Chassis(void);
void CAN1_TX_Gimbal_Pitch(void);
void CAN1_TX_Gimbal_Yaw(void);
void CAN1_TX_SuperCap(void);
void CAN2_TX_Shoot(void);
void CAN2_TX_DUAL(void);

void Encoder_Data_Process(CAN_Data_TypeDef *encoder_data, short init_angle);
#endif
