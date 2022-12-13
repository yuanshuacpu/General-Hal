#include "debug.h"

//底盘发送速度
extern int now_count;
//yaw发送速度
extern int yaw_count;
//pitch 发送速度
extern int pitch_count;
//以上三个用于查看debug

//底盘任务速度
int chassis_task_speed=3;
//云台任务速度
int gimbal_task_speed=10;
//imu任务速度
int imu_task_speed=1;
//nuc任务速度
int nuc_task_speed=2;

//扭腰速度，不要给太快
float Twist_speed=10;
//小陀螺速度
float rotate_speed=5000;