#ifndef TYPE_MODULE_H
#define TYPE_MODULE_H

#include "main.h"

/****
 * 以下为硬件数据
 * ***/
// 遥控器数据类型

extern struct DBUS_DecodingData_TypeDef DBUS;

// bmi088数据
extern struct mpu_data_t mpu_data;
extern float Yaw_Angle; // 随动，积分角度

// 电机数据

// 信号标志
extern int CAN1_Signal;     // CAN1信号量标志
extern int CAN2_Signal;     // CAN2信号量标志
extern int SuperCap_Signal; // 超级电容信号量标志
// 底盘

extern struct CAN_Data_TypeDef CAN_Chassis[4]; // 0x201 - 0x204

// 云台

extern struct CAN_Data_TypeDef CAN_Gimbal[2]; // Yaw-0x205 Pitch-0x209

// CAN2

// 发射

extern struct CAN_Data_TypeDef CAN_Trigger;  // Trigger-0x202
extern struct CAN_Data_TypeDef CAN_Shoot[2]; // Left_Fric-0x203 Right_Fric-0x204
extern int Trig_Time;
extern int Shoot_Once_Flag;
extern int Shoot_Aim_Angle;
extern int shoot_counter;

// 舵

extern struct CAN_Data_TypeDef CAN_Rudder[4]; // 0x205-0x208

// 超级电容

extern struct SuperCap_TypeDef SuperCap_Info; // SuperCap-0x210
extern int power_relay;                       // 电容充电能量
extern int power_flag;                        // 根据底盘功率设定系数

// Pid数据
// 底盘PID
extern struct PID_TypeDef PID_Chassis[4];
extern struct PID_TypeDef PID_Storage_Angle;
extern struct PID_TypeDef PID_Storage_Speed;

// 云台PID
extern struct PID_TypeDef PID_Gimbal_Angle[2];
extern struct PID_TypeDef PID_Gimbal_Speed[2];

extern struct PID_TypeDef PID_Gimbal_Angle_Auto[2];
extern struct PID_TypeDef PID_Gimbal_Speed_Auto[2];

extern struct PID_TypeDef PID_Chassis_Omega;

// 发射PID
extern struct PID_TypeDef PID_Shoot[2];
extern struct PID_TypeDef PID_Trigger[2];

// 拨弹盘单发角度环
extern struct PID_TypeDef PID_Trigger_Angle;
extern struct PID_TypeDef PID_Trigger_Speed;

// gimbal 初始化角度和中间运算的数据类型
extern struct Gimbal_Init_Processing_TypeDef Gimbal_Init_Processing_Data;

// referee_info 数据类型
extern struct Referee_Info Referee_Inf;
// 热量
extern int heat_control; // 热量控制
extern int heat_remain;  // 剩余热量

// Nuc 数据类型
extern struct NUC_Typedef NUC_Data;
extern uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE];

/****
 * 以下为任务数据
 * ***/

// 云台
extern enum Gimbal_Mode_Enum Gimbal_Mode;

// 底盘
extern enum Chassis_Mode_Enum Chassis_Mode;
extern struct Chassis_Control_Speed_Typedef rc;
extern struct Chassis_Control_Speed_Typedef keyboard;

// 发射任务数据
extern enum Shoot_Mode_Enum Shoot_Mode;
extern enum Shoot_Cmd_Enum Shoot_Cmd;

/****
 * 以下为HAL库专属数据
 * ***/

// 事件组

extern osEventFlagsId_t EventCommunicationHandle;

// HAL库所有外设的控制句柄

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#endif