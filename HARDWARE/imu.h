#ifndef __IMU_H
#define __IMU_H
#include <math.h>//��ѧ����
#include "mpu6500.h"//�������������
#include "main.h"

void IMU_Init(void);
void IMU_Data_Update(void);
void IMU_Get_Data(void);
void IMU_Ahrs_Update(void);
void IMU_Attitude_Update(void);

#endif

