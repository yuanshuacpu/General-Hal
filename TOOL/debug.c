#include "debug.h"

//���̷����ٶ�
extern int now_count;
//yaw�����ٶ�
extern int yaw_count;
//pitch �����ٶ�
extern int pitch_count;
//�����������ڲ鿴debug

//���������ٶ�
int chassis_task_speed=3;
//��̨�����ٶ�
int gimbal_task_speed=10;
//imu�����ٶ�
int imu_task_speed=1;
//nuc�����ٶ�
int nuc_task_speed=2;

//Ť���ٶȣ���Ҫ��̫��
float Twist_speed=10;
//С�����ٶ�
float rotate_speed=5000;