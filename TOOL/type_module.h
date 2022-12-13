#ifndef TYPE_MODULE_H
#define TYPE_MODULE_H

#include "main.h"

/****
 * ����ΪӲ������
 * ***/
// ң������������

extern struct DBUS_DecodingData_TypeDef DBUS;

// bmi088����
extern struct mpu_data_t mpu_data;
extern float Yaw_Angle; // �涯�����ֽǶ�

// �������

// �źű�־
extern int CAN1_Signal;     // CAN1�ź�����־
extern int CAN2_Signal;     // CAN2�ź�����־
extern int SuperCap_Signal; // ���������ź�����־
// ����

extern struct CAN_Data_TypeDef CAN_Chassis[4]; // 0x201 - 0x204

// ��̨

extern struct CAN_Data_TypeDef CAN_Gimbal[2]; // Yaw-0x205 Pitch-0x209

// CAN2

// ����

extern struct CAN_Data_TypeDef CAN_Trigger;  // Trigger-0x202
extern struct CAN_Data_TypeDef CAN_Shoot[2]; // Left_Fric-0x203 Right_Fric-0x204
extern int Trig_Time;
extern int Shoot_Once_Flag;
extern int Shoot_Aim_Angle;
extern int shoot_counter;

// ��

extern struct CAN_Data_TypeDef CAN_Rudder[4]; // 0x205-0x208

// ��������

extern struct SuperCap_TypeDef SuperCap_Info; // SuperCap-0x210
extern int power_relay;                       // ���ݳ������
extern int power_flag;                        // ���ݵ��̹����趨ϵ��

// Pid����
// ����PID
extern struct PID_TypeDef PID_Chassis[4];
extern struct PID_TypeDef PID_Storage_Angle;
extern struct PID_TypeDef PID_Storage_Speed;

// ��̨PID
extern struct PID_TypeDef PID_Gimbal_Angle[2];
extern struct PID_TypeDef PID_Gimbal_Speed[2];

extern struct PID_TypeDef PID_Gimbal_Angle_Auto[2];
extern struct PID_TypeDef PID_Gimbal_Speed_Auto[2];

extern struct PID_TypeDef PID_Chassis_Omega;

// ����PID
extern struct PID_TypeDef PID_Shoot[2];
extern struct PID_TypeDef PID_Trigger[2];

// �����̵����ǶȻ�
extern struct PID_TypeDef PID_Trigger_Angle;
extern struct PID_TypeDef PID_Trigger_Speed;

// gimbal ��ʼ���ǶȺ��м��������������
extern struct Gimbal_Init_Processing_TypeDef Gimbal_Init_Processing_Data;

// referee_info ��������
extern struct Referee_Info Referee_Inf;
// ����
extern int heat_control; // ��������
extern int heat_remain;  // ʣ������

// Nuc ��������
extern struct NUC_Typedef NUC_Data;
extern uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE];

/****
 * ����Ϊ��������
 * ***/

// ��̨
extern enum Gimbal_Mode_Enum Gimbal_Mode;

// ����
extern enum Chassis_Mode_Enum Chassis_Mode;
extern struct Chassis_Control_Speed_Typedef rc;
extern struct Chassis_Control_Speed_Typedef keyboard;

// ������������
extern enum Shoot_Mode_Enum Shoot_Mode;
extern enum Shoot_Cmd_Enum Shoot_Cmd;

/****
 * ����ΪHAL��ר������
 * ***/

// �¼���

extern osEventFlagsId_t EventCommunicationHandle;

// HAL����������Ŀ��ƾ��

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