#include "IMU.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "My_USART.h"
#include "LSM303D.h"
#include "L3GD20.h"
#include "delay.h"
#include "CommonConversions.h"

// 互补滤波成功前提是芯片能够提供20hz的低通滤波器
#if 0
void QCF_Attitude(void)
{		
    double norm;

    double ax, ay, az;
    double gx, gy, gz;

    float pitchAcc, rollAcc;
	
		static uint8_t is_init_q = false;

    static double rad_yaw = 0;
    static double rad_pitch = 0;
    static double rad_roll = 0;

    static double q0 = 1.0;
    static double q1 = 0.0;
    static double q2 = 0.0;
    static double q3 = 0.0;
	
		static double lq0 = 1.0;
    static double lq1 = 0.0;
    static double lq2 = 0.0;
    static double lq3 = 0.0;

    static double Tbn[4][4];
	
#ifdef TWO_Sample	
	
		static double rot_h[3] = {0, 0, 0};
	
		static double d_angle_gx[2] = {0, 0};
		static double d_angle_gy[2] = {0, 0};
		static double d_angle_gz[2] = {0, 0};
		
#endif
		double q00, q11, q22, q33, q01, q02, q03, q12, q13, q23;	
		
#if 0			
    static int16_t gyro_tmp[3] = {0, 0, 0}, accle_tmp[3] = {0, 0, 0};
			
		My_MPU6050_Get_Gyro_Val(gyro_tmp);
    My_MPU6050_Get_Accle_Val(accle_tmp);			
		
		gx = (double)gyro_tmp[0] / MPU6050_GYRO_DEG;
		gy = (double)gyro_tmp[1] / MPU6050_GYRO_DEG;
		gz = (double)gyro_tmp[2] / MPU6050_GYRO_DEG;
		
		gx = gx * DEG_RAD;
		gy = gy * DEG_RAD;
		gz = gz * DEG_RAD;

    gx -= Global_MPU6050_Gyro_Offset_Val[0];
    gy -= Global_MPU6050_Gyro_Offset_Val[1];
    gz -= Global_MPU6050_Gyro_Offset_Val[2];	
				
    ax = (double)accle_tmp[0];
    ay = (double)accle_tmp[1];
    az = (double)accle_tmp[2];	
#else

		ax = Global_Accel_Sensor.filtered.x;
		ay = Global_Accel_Sensor.filtered.y;
		az = Global_Accel_Sensor.filtered.z;
		
		gx = Global_Gyro_Sensor.filtered.x;
		gy = Global_Gyro_Sensor.filtered.y;
		gz = Global_Gyro_Sensor.filtered.z;
		
		if(!is_init_q)
		{
				lq0 = Global_Init_Q[0];
				lq1 = Global_Init_Q[1];
				lq2 = Global_Init_Q[2];
				lq3 = Global_Init_Q[3];
				INSSetQuaternion(Global_Init_Q);
				is_init_q = true;
		}
		
#endif
				
#ifdef ONE_Sample

		q0 = lq0 - (lq1 * gx + lq2 * gy + lq3 * gz) * dt / 2.0f;
		q1 = lq1 + (lq0 * gx + lq2 * gz - lq3 * gy) * dt / 2.0f;
		q2 = lq2 + (lq0 * gy - lq1 * gz + lq3 * gx) * dt / 2.0f;
		q3 = lq3 + (lq0 * gz + lq1 * gy - lq2 * gx) * dt / 2.0f;

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
		
		q0 = lq0 - 0.5 * (lq1 * rot_h[0] + lq2 * rot_h[1] + lq3 * rot_h[2]);
		q1 = lq1 + 0.5 * (lq0 * rot_h[0] + lq2 * rot_h[2] - lq3 * rot_h[1]);
		q2 = lq2 + 0.5 * (lq0 * rot_h[1] - lq1 * rot_h[2] + lq3 * rot_h[0]);
		q3 = lq3 + 0.5 * (lq0 * rot_h[2] + lq1 * rot_h[1] - lq2 * rot_h[0]);
		
#endif
	
		norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm;	

		q00 = q0 * q0;
    q11 = q1 * q1;
    q22 = q2 * q2;
    q33 = q3 * q3;
    q01 = q0 * q1;
    q02 = q0 * q2;
    q03 = q0 * q3;
    q12 = q1 * q2;
    q13 = q1 * q3;
    q23 = q2 * q3;

		Tbn[1][1] = q00 + q11 - q22 - q33;
		Tbn[2][2] = q00 - q11 + q22 - q33;
		Tbn[3][3] = q00 - q11 - q22 + q33;
		Tbn[1][2] = 2 * (q12 - q03);
		Tbn[1][3] = 2 * (q13 + q02);
		Tbn[2][1] = 2 * (q12 + q03);
		Tbn[2][3] = 2 * (q23 - q01);
		Tbn[3][1] = 2 * (q13 - q02);
		Tbn[3][2] = 2 * (q23 + q01);
		
		Global_roll = atan2(Tbn[3][2], Tbn[3][3]) * RAD_DEG;		
		rollAcc = atan2(ay, az) * RAD_DEG;		
		Global_roll = Global_roll * (1 - PMC) + rollAcc * PMC;		

		Global_pitch = asin(-1 * Tbn[3][1]) * RAD_DEG;
		pitchAcc = atan2(-1 * ax, az) * RAD_DEG;
		Global_pitch = Global_pitch * (1 - PMC) + pitchAcc * PMC;
		
		Global_yaw = atan2(Tbn[2][1], Tbn[1][1]) * RAD_DEG;

		rad_yaw = (double)Global_yaw * DEG_RAD;
		rad_pitch = (double)Global_pitch * DEG_RAD;
		rad_roll = (double)Global_roll * DEG_RAD;
		
		lq0 = cos(rad_roll / 2) * cos(rad_pitch / 2) * cos(rad_yaw / 2) + sin(rad_roll / 2) * sin(rad_pitch / 2) * sin(rad_yaw / 2);
		lq1 = sin(rad_roll / 2) * cos(rad_pitch / 2) * cos(rad_yaw / 2) - cos(rad_roll / 2) * sin(rad_pitch / 2) * sin(rad_yaw / 2);
		lq2 = cos(rad_roll / 2) * sin(rad_pitch / 2) * cos(rad_yaw / 2) + sin(rad_roll / 2) * cos(rad_pitch / 2) * sin(rad_yaw / 2);
		lq3 = cos(rad_roll / 2) * cos(rad_pitch / 2) * sin(rad_yaw / 2) - sin(rad_roll / 2) * sin(rad_pitch / 2) * cos(rad_yaw / 2);		
		
		Global_Q[0] = lq0;
		Global_Q[1] = lq1;
		Global_Q[2] = lq2;
		Global_Q[3] = lq3;
#if 0	
		My_USART_send_MUX_Bytes_x(Serial_3, 0x5f, Global_roll);				
		My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_pitch);
		My_USART_send_MUX_Bytes_x(Serial_3, 0x1f, accle_tmp[2]);						
#endif
}


#else

#endif
