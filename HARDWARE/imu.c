/**
 ****************************(C) COPYRIGHT 2021 SHUIMUJIEMING****************************
 * @file       imu.c/h
 * @brief      陀螺仪初始化以及角度计算等函数
 * @note
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     7-10-2021     shuimujieming              1. 完成
 *
 ****************************(C) COPYRIGHT 2021 SHUIMUJIEMING****************************
 */
#include "imu.h"

#define Kp 2.0f // 陀螺仪角度计算比例

#define Ki 0.01f // 陀螺仪角度计算积分

// #define IST8310  //定义磁力传感器是否使用

#define BOARD_DOWN 0 // 板子是否朝下

// 四元数初始化定义，使用volatile定义其为易变内存
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

volatile float exInt, eyInt, ezInt; /* error integral */

static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t last_update = 0, now_update = 0; /* Sampling cycle count, ubit ms */
static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
uint8_t mpu_buff[14]; /* buffer to save imu raw data */
uint8_t ist_buff[6];  /* buffer to save IST8310 raw data */


/*
 *@title：四元数初始化
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
void Quaternion_Init()
{
	int16_t hx, hy;

	hx = mpu_data.processed_data.mx;
	hy = mpu_data.processed_data.my;

// 判断板子是否朝下
#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
	}
	else if (hx < 0 && hy > 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
	}
	else if (hx > 0 && hy > 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
	}
	else if (hx > 0 && hy < 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}
	}
#else
	if (hx < 0 && hy < 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
	}
	else if (hx < 0 && hy > 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
	}
	else if (hx > 0 && hy > 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
	}
	else if (hx > 0 && hy < 0)
	{
		if (abs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}
	}
#endif
}
/*shuimujieming
 *@title：求解平方根倒数
 *@description：1/Sqrt(x)
 *@param 1：
 *@param 2：
 *@return:：
 */

float inv_sqrt(float x)
{
	float xhalf = 0.5f * x;			// get half of x
	int i = *(int *)&x;				// get bits for floating value 此处使用的数据类型位int，而源程序为long。注意！
	i = 0x5f3759df - (i >> 1);		// gives initial guess y0
	x = *(float *)&i;				// convert bits back to float
	x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy

	return x;
}

/*
 *@title：IMU姿态更新
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
void IMU_Ahrs_Update()
{
	float norm; // 归一化数
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;

	// 四元数乘积
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	// 读取陀螺仪9轴数据
	gx = mpu_data.processed_data.wx;
	gy = mpu_data.processed_data.wy;
	gz = mpu_data.processed_data.wz;
	ax = mpu_data.origin_data.ax;
	ay = mpu_data.origin_data.ay;
	az = mpu_data.origin_data.az;
	mx = mpu_data.processed_data.mx;
	my = mpu_data.processed_data.my;
	mz = mpu_data.processed_data.mz;

	// 时间获取，用于积分
	now_update = mpu_data.imu_tick_counts; // 1ms
	halfT = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;

	// 对加速度进行归一化
	norm = inv_sqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

// 磁力计引入
#ifdef IST8310
	norm = inv_sqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
#else
	mx = 0;
	my = 0;
	mz = 0;
#endif

	/* compute reference direction of flux */
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	/*
	 * error is sum of cross product between reference direction
	 * of fields and direction measured by sensors
	 */
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	// 比例积分
	/* PI */
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}

	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// 四元数归一化
	norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}
/*
 *@title：IMU欧拉角更新
 *@description：
 *@param 1：
 *@param 2：
 *@return:：
 */
void IMU_Attitude_Update()
{
	// 四元数转化为欧拉角
	/* roll   -pi----pi  */
	mpu_data.processed_data.rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;

	/* pitch  -pi/2----pi/2 */
	mpu_data.processed_data.pit = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;

	/* yaw    -pi----pi */
	mpu_data.processed_data.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
	//	imu.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3;
}
