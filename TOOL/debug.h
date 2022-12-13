#ifndef __DEBUG_H
#define __DEBUG_H
#include "main.h"

//这三个只是发送数据为0 但不会停止发送
#define CHASSIS_OFF 0

#define PITCH_OFF 1

#define YAW_OFF 0

//这三个直接停止发送
#define CHASSIS_SEND_OFF 0

#define PITCH_SEND_OFF 0

#define YAW_SEND_OFF 0

//调节舵机使用
#define COVER_OPEN_VAL 800
#define COVER_CLOSE_VAL 950

//可以挂起云台或者底盘任务
#define CHASSIS_SUSPEND 0

#define GIMBAL_SUSPEND  0


#endif