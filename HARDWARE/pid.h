#ifndef __PID_H
#define __PID_H
#include "main.h"

typedef struct PID_TypeDef PID_TypeDef;

void Pid_Set(PID_TypeDef *PID, float Kp, float Ki, float Kd, float Pmax, float Imax, float Dmax, float PIDmax, float Speed_Ratio, float Acceletation, float Error_Ignored, u8 Motor_Type);
float Pid_Calc(PID_TypeDef *PID, float Current_Speed, float Target_Current);
void Pid_Reset(PID_TypeDef *PID);
void Chassis_PID_Init();
void Gimbal_PID_Init();
void Shoot_PID_Init();
void PID_Init();
#endif