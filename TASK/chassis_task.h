#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
typedef struct
{
	float vx;
	float vy;
	float vw;
	short wheel_speed[4];
} Chassis_Speed_Typedef;

#include "main.h"
// 3
#define MAX_WHEEL_SPEED 8500

void Get_Chassis_RC(void);

#endif