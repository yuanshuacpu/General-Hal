#include "type_module.h"
/****
 * 以下为硬件数据
 * ***/
// 遥控器数据
struct DBUS_DecodingData_TypeDef DBUS;

// bmi088数据

struct mpu_data_t mpu_data;
float Yaw_Angle; // 随动，积分角度

// 电机数据

int CAN1_Signal = 0;     // CAN1信号量标志
int CAN2_Signal = 0;     // CAN2信号量标志
int SuperCap_Signal = 0; // 超级电容信号量标志

// 底盘

struct CAN_Data_TypeDef CAN_Chassis[4]; // 0x201 - 0x204
// 云台
struct CAN_Data_TypeDef CAN_Gimbal[2]; // Yaw-0x205 Pitch-0x209

// CAN2
// 发射
struct CAN_Data_TypeDef CAN_Trigger;  // Trigger-0x202
struct CAN_Data_TypeDef CAN_Shoot[2]; // Left_Fric-0x203 Right_Fric-0x204
int Trig_Time = 0;                    // 发射触发时间
int Shoot_Once_Flag = 0;              // 一次只发送一次的标志位
int Shoot_Aim_Angle = 0;
int shoot_counter = 0; // 子弹计数

// 舵
struct CAN_Data_TypeDef CAN_Rudder[4]; // 0x205-0x208

// 超级电容
struct SuperCap_TypeDef SuperCap_Info; // SuperCap-0x210
int power_relay = 0;                   // 电容充电能量
int power_flag = 0;                    // 根据底盘功率设定系数

// Pid数据
// 底盘PID
struct PID_TypeDef PID_Chassis[4];
struct PID_TypeDef PID_Storage_Angle;
struct PID_TypeDef PID_Storage_Speed;

// 云台PID
struct PID_TypeDef PID_Gimbal_Angle[2];
struct PID_TypeDef PID_Gimbal_Speed[2];
struct PID_TypeDef PID_Gimbal_Angle_Stop[2];
struct PID_TypeDef PID_Gimbal_Speed_Stop[2];

struct PID_TypeDef PID_Gimbal_Angle_Auto[2];
struct PID_TypeDef PID_Gimbal_Speed_Auto[2];

struct PID_TypeDef PID_Chassis_Omega;

// 发射PID
struct PID_TypeDef PID_Shoot[2];
struct PID_TypeDef PID_Trigger[2];

// 拨弹盘单发角度环
struct PID_TypeDef PID_Trigger_Angle;
struct PID_TypeDef PID_Trigger_Speed;

// gimbal 初始化角度和中间运算的数据类型
struct Gimbal_Init_Processing_TypeDef Gimbal_Init_Processing_Data = {0};

// referee_info 数据类型
struct Referee_Info Referee_Inf = {0};
int heat_control = 4; // 热量控制
int heat_remain = 0;  // 剩余热量

// Nuc 数据类型
struct NUC_Typedef NUC_Data;
uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE];

/****
 * 以下为任务数据
 * ***/

// 云台数据
enum Gimbal_Mode_Enum Gimbal_Mode;

// 底盘数据
enum Chassis_Mode_Enum Chassis_Mode;
struct Chassis_Control_Speed_Typedef rc;
struct Chassis_Control_Speed_Typedef keyboard;

// 发射任务数据
enum Shoot_Mode_Enum Shoot_Mode;
enum Shoot_Cmd_Enum Shoot_Cmd = SHOOT_CMD_STOP;
