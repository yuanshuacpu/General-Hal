#ifndef TYPE_DEFINE_H
#define TYPE_DEFINE_H
#include "main.h"
/****
 * ����ΪӲ����������
 * ***/
// ң���� �ṹ��

struct DBUS_DecodingData_TypeDef
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        uint8_t Switch_Left;
        uint8_t Switch_Right;
    } RC;
    struct
    {
        int16_t X;
        // int16_t X_Real[6];
        int16_t Y;
        // int16_t Y_Real[6];
        uint8_t Press_Left;
        uint8_t Press_Right;
        uint16_t Keyboard;
    } PC;

    int DR16_Signal;
    /*  struct
     {
         int16_t ch0;
         int16_t ch1;
         int16_t ch2;
         int16_t ch3;
         int16_t ch4;
         int16_t omega;
         int16_t angle;
     } Control; */
};

// bmi088��������
struct mpu_data_t
{
    struct
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        int16_t mx;
        int16_t my;
        int16_t mz;

        int16_t temp;

        int16_t gx;
        int16_t gy;
        int16_t gz;

        int16_t ax_offset;
        int16_t ay_offset;
        int16_t az_offset;

        int16_t gx_offset;
        int16_t gy_offset;
        int16_t gz_offset;
    } origin_data;

    struct
    {
        int16_t mx;
        int16_t my;
        int16_t mz;

        float temp;

        float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
        float wy;
        float wz;

        float vx;
        float vy;
        float vz;

        float rol;
        float pit;
        float yaw;
    } processed_data;

    uint32_t imu_tick_counts;
};

// can��������
struct CAN_Data_TypeDef
{
    short Current_Speed;
    short Current_MechAngle;
    short Current;
    short Temperature;
    short Critical_MechAngle;
    short Init_MechAngle;
    short Origin_MechAngle;
    short Target_Current;
    short Target_MechAngle;
    short Last_MechAngle;
    int Total_MechAngle;
};

// ��������can��������
struct SuperCap_TypeDef
{
    float InputVot;
    float CapVot;
    float InputCurrent;
    float CurrentPower;
    float Low_Filter_Vot;
    int id;
};

// PID ��������
struct PID_TypeDef
{
    float P;
    float I;
    float D;
    float CurrentError;
    float LastError;
    float ErrorIgnored;
    float Pout;
    float Iout;
    float Dout;
    float PIDout;
    float PIDOutCompensate;
    float PMax;
    float IMax;
    float DMax;
    float PIDOutMax;
    float PIDOutLast;
    float Target_Speed_Last;
    float Speed_Ratio;
    float Acceleration;
    u8 Motor_Type;
};

// gimbal ��ʼ���ǶȺ��м��������������
struct Gimbal_Init_Processing_TypeDef
{
    // ��һ��ֵ�Ƕ�
    short Angle_Last;
    // ��ʼ�Ƕ�
    short Init_Angle;
    // ԭʼYaw��
    short Origin_Init_Yaw_Angle;
    // ԭʼPitch��
    short Origin_Init_Pitch_Angle;

    // Pitch��������������
    float Pitch_Aim_Angle;
    float Pitch_Angle_Max;
    float Pitch_Angle_Min;

    // Yaw��������������
    float Yaw_Aim_Angle;
};

// referee_info ��������
struct Referee_Info
{
    char Robot_Color;
    // ����״̬���ݣ�0X0001  1HZ
    struct
    {
        uint8_t game_type : 4;      // ��������
        uint8_t game_progress : 4;  // ��ǰ�����׶�
        uint16_t stage_remain_time; // ��ǰ�׶�ʣ��ʱ��
        uint64_t SyncTimeStamp;     // �����˽��յ���ָ��ľ�ȷ Unix ʱ�䣬�����ض��յ���Ч�� NTP ��������ʱ����Ч

    } ext_game_status_t;

    // ����������ݣ�0x0002  ������������
    struct
    {
        uint8_t winner; // 0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��

    } ext_game_result_t;

    // ������Ѫ������:0x0003  1Hz
    struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP; // �췽ǰ��սѪ��
        uint16_t red_base_HP;    // �췽����Ѫ��
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP; // ����ǰ��վѪ��
        uint16_t blue_base_HP;    // ��������Ѫ��
    } robot_hp;

    // ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л�����
    struct
    {
        uint8_t dart_belong;           // ������ڵĶ��飺1���췽���� 2����������
        uint16_t stage_remaining_time; // ����ʱ��ʣ�����ʱ�䣬��λs
    } ext_dart_status_t;

    // �����¼����ݣ� 0x0101  �¼��ı����
    struct
    {
        uint32_t event_type;
    } ext_event_data_t;

    // ����վ������ʶ��0x0102   �����ı����
    struct
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } ext_supply_projectile_action_t;

    // ����ϵͳ������Ϣ��0x0104 �������
    struct
    {
        uint8_t level;         // ����ȼ�
        uint8_t foul_robot_id; // ��������� ID
    } ext_referee_warning_t;

    // ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������
    struct
    {
        uint8_t dart_remaining_time;
    } ext_dart_remaining_time_t;

    // ����������״̬��0x0201  10Hz
    struct
    {
        uint8_t robot_id;                        // ������ID
        uint8_t robot_level;                     // �����˵ȼ�
        uint16_t remain_HP;                      // ������ʣ��Ѫ��
        uint16_t max_HP;                         // ����������Ѫ��
        uint16_t shooter_id1_17mm_cooling_rate;  // ������ 1 �� 17mm ǹ��ÿ����ȴֵ
        uint16_t shooter_id1_17mm_cooling_limit; // ������ 1 �� 17mm ǹ����������
        uint16_t shooter_id1_17mm_speed_limit;   // ������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
        uint16_t shooter_id2_17mm_cooling_rate;  // ������ 2 �� 17mm ǹ��ÿ����ȴֵ
        uint16_t shooter_id2_17mm_cooling_limit; // ������ 2 �� 17mm ǹ����������
        uint16_t shooter_id2_17mm_speed_limit;   // ������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
        uint16_t shooter_id1_42mm_cooling_rate;  // ������ 42mm ǹ��ÿ����ȴֵ
        uint16_t shooter_id1_42mm_cooling_limit; // ������ 42mm ǹ����������
        uint16_t shooter_id1_42mm_speed_limit;   // ������ 42mm ǹ�������ٶ� ��λ m/s
        uint16_t chassis_power_limit;            // �����˵��̹�����������
        uint8_t mains_power_gimbal_output : 1;   // ���ص�Դ��������1Ϊ24v��0Ϊ�����
        uint8_t mains_power_chassis_output : 1;
        uint8_t mains_power_shooter_output : 1;
    } game_robot_state;
    // ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz
    struct
    {
        uint16_t chassis_volt;                  // ���������ѹ ��λ ����
        uint16_t chassis_current;               // ����������� ��λ ����
        float chassis_power;                    // ����������� ��λ W ��
        uint16_t chassis_power_buffer;          // ���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
        uint16_t shooter_id1_17mm_cooling_heat; // 1 �� 17mm ǹ������
        uint16_t shooter_id2_17mm_cooling_heat; // 2 �� 17mm ǹ������
        uint16_t shooter_id1_42mm_cooling_heat; // 42mm ǹ������
    } power_heat_data;
    // ������λ�ã�0x0203 10Hz
    struct
    {
        float x;
        float y;
        float z;
        float yaw;
    } ext_game_robot_pos_t;

    // ���������棺0x0204������Ƶ�ʣ�1Hz
    /*
    bit 0��������Ѫ����Ѫ״̬
    bit 1��ǹ��������ȴ����
    bit 2�������˷����ӳ�
    bit 3�������˹����ӳ�
    ���� bit ����
    */
    struct
    {
        uint8_t power_rune_buff;
    } buff_data;

    // ���л���������״̬��0x0205  10Hz
    struct
    {
        uint8_t attack_time;
    } aerial_robot_energy_t;

    // �˺�״̬��0x0206    �˺���������
    struct
    {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    } robot_hurt_data;

    // �����Ϣ��0x0207  �������
    struct
    {
        uint8_t bullet_type; // �ӵ�����: 1��17mm ���� 2��42mm ����
        uint8_t shooter_id;  // ������� ID��1��1 �� 17mm �������2��2 �� 17mm ������� 3��42mm �������
        uint8_t bullet_freq; // �ӵ���Ƶ ��λ Hz
        float bullet_speed;  // �ӵ����� ��λ m/s
    } real_shoot_data;

    // �ӵ�ʣ�෢����Ŀ��0x0208 1HZ
    struct
    {
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    } bullet_remaining;

    // ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ������
    struct
    {
        uint32_t rfid_status;
    } ext_rfid_status_t;
    // ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
    struct
    {
        uint8_t dart_launch_opening_status;
        uint8_t dart_attack_target;
        uint16_t target_change_time;
        uint8_t first_dart_speed;
        uint8_t second_dart_speed;
        uint8_t third_dart_speed;
        uint8_t fourth_dart_speed;
        uint16_t last_dart_launch_time;
        uint16_t operate_launch_cmd_time;
    } ext_dart_client_cmd_t;

    struct
    {
        float target_position_x;
        float target_position_y;
        float target_position_z;
        uint8_t commd_keyboard;
        uint16_t target_robot_ID;
    } robot_command_data;
};

// Nuc ��������
struct NUC_Typedef
{
    int8_t Armor_Type;    // װ������  0-��  1-С  2-��
    float Yaw_Angle;      // Y�������Ƕ�
    float Pitch_Angle;    // P�������Ƕ�
    short Enemy_distance; // �з�����
    char Armor_ID;        // �з�װ�׺���
    short Enemy_Real_Speed;
    char NUC_Shoot_Allow_Flag;
    char Auto_Shoot;
};

/****
 * ����Ϊ������������
 * ***/

// ��̨������������
enum Gimbal_Mode_Enum
{
    GIMBAL_MODE_STOP = 0,
    GIMBAL_MODE_ROTATE,
    GIMBAL_MODE_FOLLOW,
    GIMBAL_MODE_AUTOAIM,
    GIMBAL_MODE_KEEP,
    GIMBAL_MODE_RUNE
};

// ����������������
struct Chassis_Control_Speed_Typedef
{
    float vx;
    float vy;
    float vw;
};
enum Chassis_Mode_Enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_ROTATE,
    CHASSIS_MODE_FOLLOW,
    CHASSIS_MODE_AUTOAIM,
    CHASSIS_MODE_SUPERCAP,
    CHASSIS_MODE_RUNE

};

// ����������������

enum Shoot_Mode_Enum
{
    SHOOT_MODE_STOP = 0,
    SHOOT_MODE_RUN,
};
enum Shoot_Cmd_Enum
{
    SHOOT_CMD_STOP = 0,
    SHOOT_CMD_ONCE,
    SHOOT_CMD_LONG,
};

#endif