#ifndef TYPE_DEFINE_H
#define TYPE_DEFINE_H
#include "main.h"
/****
 * 以下为硬件数据类型
 * ***/
// 遥控器 结构体

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

// bmi088数据类型
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

// can数据类型
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

// 超级电容can数据类型
struct SuperCap_TypeDef
{
    float InputVot;
    float CapVot;
    float InputCurrent;
    float CurrentPower;
    float Low_Filter_Vot;
    int id;
};

// PID 数据类型
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

// gimbal 初始化角度和中间运算的数据类型
struct Gimbal_Init_Processing_TypeDef
{
    // 上一中值角度
    short Angle_Last;
    // 初始角度
    short Init_Angle;
    // 原始Yaw角
    short Origin_Init_Yaw_Angle;
    // 原始Pitch角
    short Origin_Init_Pitch_Angle;

    // Pitch轴运算数据类型
    float Pitch_Aim_Angle;
    float Pitch_Angle_Max;
    float Pitch_Angle_Min;

    // Yaw轴运算数据类型
    float Yaw_Aim_Angle;
};

// referee_info 数据类型
struct Referee_Info
{
    char Robot_Color;
    // 比赛状态数据：0X0001  1HZ
    struct
    {
        uint8_t game_type : 4;      // 比赛类型
        uint8_t game_progress : 4;  // 当前比赛阶段
        uint16_t stage_remain_time; // 当前阶段剩余时间
        uint64_t SyncTimeStamp;     // 机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效

    } ext_game_status_t;

    // 比赛结果数据：0x0002  比赛结束后发送
    struct
    {
        uint8_t winner; // 0 平局 1 红方胜利 2 蓝方胜利

    } ext_game_result_t;

    // 机器人血量数据:0x0003  1Hz
    struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP; // 红方前哨战血量
        uint16_t red_base_HP;    // 红方基地血量
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP; // 蓝方前哨站血量
        uint16_t blue_base_HP;    // 蓝方基地血量
    } robot_hp;

    // 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人
    struct
    {
        uint8_t dart_belong;           // 发射飞镖的队伍：1：红方飞镖 2：蓝方飞镖
        uint16_t stage_remaining_time; // 发射时的剩余比赛时间，单位s
    } ext_dart_status_t;

    // 场地事件数据： 0x0101  事件改变后发送
    struct
    {
        uint32_t event_type;
    } ext_event_data_t;

    // 补给站动作标识：0x0102   动作改变后发送
    struct
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } ext_supply_projectile_action_t;

    // 裁判系统警告信息：0x0104 警告后发送
    struct
    {
        uint8_t level;         // 警告等级
        uint8_t foul_robot_id; // 犯规机器人 ID
    } ext_referee_warning_t;

    // 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人
    struct
    {
        uint8_t dart_remaining_time;
    } ext_dart_remaining_time_t;

    // 比赛机器人状态：0x0201  10Hz
    struct
    {
        uint8_t robot_id;                        // 机器人ID
        uint8_t robot_level;                     // 机器人等级
        uint16_t remain_HP;                      // 机器人剩余血量
        uint16_t max_HP;                         // 机器人上限血量
        uint16_t shooter_id1_17mm_cooling_rate;  // 机器人 1 号 17mm 枪口每秒冷却值
        uint16_t shooter_id1_17mm_cooling_limit; // 机器人 1 号 17mm 枪口热量上限
        uint16_t shooter_id1_17mm_speed_limit;   // 机器人 1 号 17mm 枪口上限速度 单位 m/s
        uint16_t shooter_id2_17mm_cooling_rate;  // 机器人 2 号 17mm 枪口每秒冷却值
        uint16_t shooter_id2_17mm_cooling_limit; // 机器人 2 号 17mm 枪口热量上限
        uint16_t shooter_id2_17mm_speed_limit;   // 机器人 2 号 17mm 枪口上限速度 单位 m/s
        uint16_t shooter_id1_42mm_cooling_rate;  // 机器人 42mm 枪口每秒冷却值
        uint16_t shooter_id1_42mm_cooling_limit; // 机器人 42mm 枪口热量上限
        uint16_t shooter_id1_42mm_speed_limit;   // 机器人 42mm 枪口上限速度 单位 m/s
        uint16_t chassis_power_limit;            // 机器人底盘功率限制上限
        uint8_t mains_power_gimbal_output : 1;   // 主控电源输出情况：1为24v，0为无输出
        uint8_t mains_power_chassis_output : 1;
        uint8_t mains_power_shooter_output : 1;
    } game_robot_state;
    // 实时功率热量数据：0x0202。发送频率：50Hz
    struct
    {
        uint16_t chassis_volt;                  // 底盘输出电压 单位 毫伏
        uint16_t chassis_current;               // 底盘输出电流 单位 毫安
        float chassis_power;                    // 底盘输出功率 单位 W 瓦
        uint16_t chassis_power_buffer;          // 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
        uint16_t shooter_id1_17mm_cooling_heat; // 1 号 17mm 枪口热量
        uint16_t shooter_id2_17mm_cooling_heat; // 2 号 17mm 枪口热量
        uint16_t shooter_id1_42mm_cooling_heat; // 42mm 枪口热量
    } power_heat_data;
    // 机器人位置：0x0203 10Hz
    struct
    {
        float x;
        float y;
        float z;
        float yaw;
    } ext_game_robot_pos_t;

    // 机器人增益：0x0204。发送频率：1Hz
    /*
    bit 0：机器人血量补血状态
    bit 1：枪口热量冷却加速
    bit 2：机器人防御加成
    bit 3：机器人攻击加成
    其他 bit 保留
    */
    struct
    {
        uint8_t power_rune_buff;
    } buff_data;

    // 空中机器人能量状态：0x0205  10Hz
    struct
    {
        uint8_t attack_time;
    } aerial_robot_energy_t;

    // 伤害状态：0x0206    伤害发生后发送
    struct
    {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    } robot_hurt_data;

    // 射击信息：0x0207  射击后发送
    struct
    {
        uint8_t bullet_type; // 子弹类型: 1：17mm 弹丸 2：42mm 弹丸
        uint8_t shooter_id;  // 发射机构 ID：1：1 号 17mm 发射机构2：2 号 17mm 发射机构 3：42mm 发射机构
        uint8_t bullet_freq; // 子弹射频 单位 Hz
        float bullet_speed;  // 子弹射速 单位 m/s
    } real_shoot_data;

    // 子弹剩余发射数目：0x0208 1HZ
    struct
    {
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    } bullet_remaining;

    // 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人
    struct
    {
        uint32_t rfid_status;
    } ext_rfid_status_t;
    // 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
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

// Nuc 数据类型
struct NUC_Typedef
{
    int8_t Armor_Type;    // 装甲类型  0-无  1-小  2-大
    float Yaw_Angle;      // Y轴增量角度
    float Pitch_Angle;    // P轴增量角度
    short Enemy_distance; // 敌方距离
    char Armor_ID;        // 敌方装甲号码
    short Enemy_Real_Speed;
    char NUC_Shoot_Allow_Flag;
    char Auto_Shoot;
};

/****
 * 以下为任务数据类型
 * ***/

// 云台任务数据类型
enum Gimbal_Mode_Enum
{
    GIMBAL_MODE_STOP = 0,
    GIMBAL_MODE_ROTATE,
    GIMBAL_MODE_FOLLOW,
    GIMBAL_MODE_AUTOAIM,
    GIMBAL_MODE_KEEP,
    GIMBAL_MODE_RUNE
};

// 底盘任务数据类型
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

// 发射任务数据类型

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