#include "type_module.h"
/****
 * ����ΪӲ������
 * ***/
// ң��������
struct DBUS_DecodingData_TypeDef DBUS;

// bmi088����

struct mpu_data_t mpu_data;
float Yaw_Angle; // �涯�����ֽǶ�

// �������

int CAN1_Signal = 0;     // CAN1�ź�����־
int CAN2_Signal = 0;     // CAN2�ź�����־
int SuperCap_Signal = 0; // ���������ź�����־

// ����

struct CAN_Data_TypeDef CAN_Chassis[4]; // 0x201 - 0x204
// ��̨
struct CAN_Data_TypeDef CAN_Gimbal[2]; // Yaw-0x205 Pitch-0x209

// CAN2
// ����
struct CAN_Data_TypeDef CAN_Trigger;  // Trigger-0x202
struct CAN_Data_TypeDef CAN_Shoot[2]; // Left_Fric-0x203 Right_Fric-0x204
int Trig_Time = 0;                    // ���䴥��ʱ��
int Shoot_Once_Flag = 0;              // һ��ֻ����һ�εı�־λ
int Shoot_Aim_Angle = 0;
int shoot_counter = 0; // �ӵ�����

// ��
struct CAN_Data_TypeDef CAN_Rudder[4]; // 0x205-0x208

// ��������
struct SuperCap_TypeDef SuperCap_Info; // SuperCap-0x210
int power_relay = 0;                   // ���ݳ������
int power_flag = 0;                    // ���ݵ��̹����趨ϵ��

// Pid����
// ����PID
struct PID_TypeDef PID_Chassis[4];
struct PID_TypeDef PID_Storage_Angle;
struct PID_TypeDef PID_Storage_Speed;

// ��̨PID
struct PID_TypeDef PID_Gimbal_Angle[2];
struct PID_TypeDef PID_Gimbal_Speed[2];
struct PID_TypeDef PID_Gimbal_Angle_Stop[2];
struct PID_TypeDef PID_Gimbal_Speed_Stop[2];

struct PID_TypeDef PID_Gimbal_Angle_Auto[2];
struct PID_TypeDef PID_Gimbal_Speed_Auto[2];

struct PID_TypeDef PID_Chassis_Omega;

// ����PID
struct PID_TypeDef PID_Shoot[2];
struct PID_TypeDef PID_Trigger[2];

// �����̵����ǶȻ�
struct PID_TypeDef PID_Trigger_Angle;
struct PID_TypeDef PID_Trigger_Speed;

// gimbal ��ʼ���ǶȺ��м��������������
struct Gimbal_Init_Processing_TypeDef Gimbal_Init_Processing_Data = {0};

// referee_info ��������
struct Referee_Info Referee_Inf = {0};
int heat_control = 4; // ��������
int heat_remain = 0;  // ʣ������

// Nuc ��������
struct NUC_Typedef NUC_Data;
uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE];

/****
 * ����Ϊ��������
 * ***/

// ��̨����
enum Gimbal_Mode_Enum Gimbal_Mode;

// ��������
enum Chassis_Mode_Enum Chassis_Mode;
struct Chassis_Control_Speed_Typedef rc;
struct Chassis_Control_Speed_Typedef keyboard;

// ������������
enum Shoot_Mode_Enum Shoot_Mode;
enum Shoot_Cmd_Enum Shoot_Cmd = SHOOT_CMD_STOP;
