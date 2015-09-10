#include "QCF_Attitude.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "My_USART.h"
#include "LSM303D.h"
#include "L3GD20.h"
#include "CommonConversions.h"
#include "CommonPlan.h"

#if 0
		#define ONE_Sample //单子样
#else
		#define TWO_Sample //双子样
#endif

#define dt             Deal_Period_ms / 1000.0f 
#define PMC            0.01f

void QCF_Attitude(void) 
{
		static float q[4] = {1, 0, 0, 0};
		static float last_q[4] = {1, 0, 0, 0};
		float euler[3];
		float accel_roll;
		float accel_pitch;
	
		double norm;

    double ax, ay, az;
    double gx, gy, gz;
	
#ifdef TWO_Sample	
	
		static double rot_h[3] = {0, 0, 0};
	
		static double d_angle_gx[2] = {0, 0};
		static double d_angle_gy[2] = {0, 0};
		static double d_angle_gz[2] = {0, 0};
		
#endif	

		ax = Global_Accel_Sensor.filtered.x;
		ay = Global_Accel_Sensor.filtered.y;
		az = Global_Accel_Sensor.filtered.z;
		
		gx = Global_Gyro_Sensor.filtered.x;
		gy = Global_Gyro_Sensor.filtered.y;
		gz = Global_Gyro_Sensor.filtered.z;
		
#ifdef ONE_Sample

		q[0] = last_q[0] - (last_q[1] * gx + last_q[2] * gy + last_q[3] * gz) * dt / 2.0f;
		q[1] = last_q[1] + (last_q[0] * gx + last_q[2] * gz - last_q[3] * gy) * dt / 2.0f;
		q[2] = last_q[2] + (last_q[0] * gy - last_q[1] * gz + last_q[3] * gx) * dt / 2.0f;
		q[3] = last_q[3] + (last_q[0] * gz + last_q[1] * gy - last_q[2] * gx) * dt / 2.0f;

#elif defined TWO_Sample

		d_angle_gx[0] = gx * dt / 2.0f;
		d_angle_gy[0] = gy * dt / 2.0f;
		d_angle_gz[0] = gz * dt / 2.0f;
		
		d_angle_gx[1] = gx * dt / 2.0f;
		d_angle_gy[1] = gy * dt / 2.0f;
		d_angle_gz[1] = gz * dt / 2.0f;
		
		rot_h[0] = d_angle_gx[0] + d_angle_gx[1] + 2.0f / 3.0f * (d_angle_gy[0] * d_angle_gz[1] - d_angle_gy[1] * d_angle_gz[0]);
		rot_h[1] = d_angle_gy[0] + d_angle_gy[1] + 2.0f / 3.0f * (d_angle_gx[1] * d_angle_gz[0] - d_angle_gx[0] * d_angle_gz[1]);
		rot_h[2] = d_angle_gz[0] + d_angle_gz[1] + 2.0f / 3.0f * (d_angle_gx[0] * d_angle_gy[1] - d_angle_gx[1] * d_angle_gy[0]);
		
		q[0] = last_q[0] - 0.5 * (last_q[1] * rot_h[0] + last_q[2] * rot_h[1] + last_q[3] * rot_h[2]);
		q[1] = last_q[1] + 0.5 * (last_q[0] * rot_h[0] + last_q[2] * rot_h[2] - last_q[3] * rot_h[1]);
		q[2] = last_q[2] + 0.5 * (last_q[0] * rot_h[1] - last_q[1] * rot_h[2] + last_q[3] * rot_h[0]);
		q[3] = last_q[3] + 0.5 * (last_q[0] * rot_h[2] + last_q[1] * rot_h[1] - last_q[2] * rot_h[0]);
		
#endif

		norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] = q[0] / norm;
		q[1] = q[1] / norm;
		q[2] = q[2] / norm;
		q[3] = q[3] / norm;
		Quaternion2Euler(q, euler);
		
		accel_roll = atan2(ay, az) * RAD_DEG;
		euler[0] = euler[0] * (1 - PMC) + accel_roll * PMC;
		
		accel_pitch = atan2(-1 * ax, az) * RAD_DEG;
		euler[1] = euler[1] * (1 - PMC) + accel_pitch * PMC;
		
		Euler2Quaternion(euler, last_q);
#if 1
		Global_Now_Euler[0] = euler[0];
		Global_Now_Euler[1] = euler[1];
		Global_Now_Euler[2] = euler[2];
#endif
}

