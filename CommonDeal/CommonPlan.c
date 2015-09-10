#include "CommonPlan.h"
#include "CommonConversions.h"
#include "My_LED.h"
#include "LSM303D.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "GPS_NEO_M8N.h"
#include "My_Flash.h"

static uint16_t Rx_LEN = 0;	
static double latRef = 0;
static double lonRef = 0;
static float altRef = 0;
static double latitudeTMP;	
static double longitudeTMP;
float altitudeTMP;
static uint16_t gps_Ref_Sample_times = 1;
static uint16_t now_times = 0;
static float last_Vel_NED[2] = {0, 0};

static float transTMP[24];
static float midTmp[16];
static float invTmp[16];
static float midTmp2[24];

static uint32_t Flash_wr[18];

static float magTmp1[3] = {0, 0, 0};
static float magTmp2[3] = {0, 0, 0};
static float calMagY = 0, calMagX = 0;

//-----------------------------------------------------------------
//函数名称：enable_interrupts
//功能概要：开总中断
//函数返回：void
//参数说明：void
//-----------------------------------------------------------------
void My_STM32_enable_interrupts(void)
{
    __set_PRIMASK(0);
}

//-----------------------------------------------------------------
//函数名称：disable_interrupts
//功能概要：关总中断
//函数返回：void
//参数说明：void
//-----------------------------------------------------------------
void My_STM32_disable_interrupts(void)
{
    __set_PRIMASK(1);
}

void Get_AHRS_Sensor_Val(void)
{
		My_MPU6050_Get_Accle_Val(Global_Accle_Val);						
		My_MPU6050_Get_Gyro_Val(Global_Gyro_Val);

		Global_Accel_Sensor.filtered.x = (float)(Global_Accle_Val[0] - Global_Accel_Sensor.offset.x) / 1638.4f; // unit m/s^2
		Global_Accel_Sensor.filtered.y = (float)(Global_Accle_Val[1] - Global_Accel_Sensor.offset.y) / 1638.4f; // unit m/s^2
		Global_Accel_Sensor.filtered.z = (float)(Global_Accle_Val[2] - Global_Accel_Sensor.offset.z) / 1638.4f; // unit m/s^2
	
		Global_Gyro_Sensor.filtered.x = (float)Global_Gyro_Val[0] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[0]; // unit rad/s
		Global_Gyro_Sensor.filtered.y = (float)Global_Gyro_Val[1] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[1]; // unit rad/s
		Global_Gyro_Sensor.filtered.z = (float)Global_Gyro_Val[2] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[2]; // unit rad/s
	
#if defined USE_MAG_LSM303D

		My_LSM303D_Get_Mag_Val(Global_Mag_Val);

#elif defined USE_MAG_HMC5883L

		MY_HMC5883L_Get_mag_Val(Global_Mag_Val);

#endif
}

/***********通过加速度计和磁力计计算参考欧拉角*************/
void Get_Ref_Euler(const float accVal[3], const float magVal[3], Magnetometer_Calibration_Struct *SensorBpTmp, float *euler)
{		
		euler[0] = atan2(accVal[1], accVal[2]); //计算Roll (-PI < Roll < PI)
		euler[1] = atan2(-1 * accVal[0], accVal[1] * sin(euler[0]) + accVal[2] * cos(euler[0])); //计算Pitch (-PI/2 < Pitch < PI/2)

#if 1	
		magTmp1[0] = magVal[0] - (*SensorBpTmp).Bp_Hard_Iron_V[0]; //去除磁力计Hard-iron effect Vx
		magTmp1[1] = magVal[1] - (*SensorBpTmp).Bp_Hard_Iron_V[1]; //去除磁力计Hard-iron effect Vy
		magTmp1[2] = magVal[2] - (*SensorBpTmp).Bp_Hard_Iron_V[2]; //去除磁力计Hard-iron effect Vz
	
		ML_R_X_ML_R((*SensorBpTmp).inv_M, magTmp1, magTmp2, 3, 3, 3, 1); //去除磁力计Soft-iron effect inv_M
#else
		magTmp1[0] = magVal[0];
		magTmp1[1] = magVal[1];
		magTmp1[2] = magVal[2];
		ML_R_X_ML_R((*SensorBpTmp).inv_M, magTmp1, magTmp2, 3, 3, 3, 1); //去除磁力计Soft-iron effect inv_M
		magTmp2[0] = magTmp2[0] - (*SensorBpTmp).Bp_Hard_Iron_V[0]; //去除磁力计Hard-iron effect Vx
		magTmp2[1] = magTmp2[1] - (*SensorBpTmp).Bp_Hard_Iron_V[1]; //去除磁力计Hard-iron effect Vy
		magTmp2[2] = magTmp2[2] - (*SensorBpTmp).Bp_Hard_Iron_V[2]; //去除磁力计Hard-iron effect Vz
#endif
		
	
		calMagY = magTmp2[2] * sin(euler[0]) - magTmp2[1] * cos(euler[0]); //倾斜补偿磁力计的Y轴分量
		calMagX = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1]) * sin(euler[0]) + magTmp2[2] * sin(euler[1]) * cos(euler[0]); //倾斜补偿磁力计的X轴分量
	
		euler[2] = atan2(calMagY, calMagX) * RAD_DEG; //计算Yaw (-PI < Roll < PI) 并将弧度转化成角度
		euler[1] *= RAD_DEG; //将弧度转化为角度
		euler[0] *= RAD_DEG; //将弧度转化为角度
	
		if(euler[2] > 180)
		{
				euler[2] -= 360;
		}
		else if(euler[2] < -180)
		{
				euler[2] += 360;
		}
}

void HomeLocationPrepare(void)
{
		static const uint16_t Sensors_Sample_Times = 100;
	
		static float q[4] = {1, 0, 0, 0}, accels[3] = {0, 0, 0}, mag[3] = {0, 0, 0}, euler[3] = {0, 0, 0}; 		
		static float alt_Ref = 0;

		static uint16_t sampleTimes = 0;
		
		float q_norm = 1, mag_norm = 1;
#if defined USE_MAG															
    float Mx, My;																	
#endif
		if(!Global_HOME_Check_NONE)
		{
				My_MPU6050_Get_Accle_Val(Global_Accle_Val);
				accels[0] += (float)Global_Accle_Val[0];//ax
				accels[1] += (float)Global_Accle_Val[1];//ay
				accels[2] += (float)Global_Accle_Val[2];//az					

		#if defined USE_MAG_LSM303D
				My_LSM303D_Get_Mag_Val(Global_Mag_Val);
		#elif defined USE_MAG_HMC5883L
				MY_HMC5883L_Get_mag_Val(Global_Mag_Val);				
		#endif
			
				mag[0] += (float)Global_Mag_Val[0];//mx
				mag[1] += (float)Global_Mag_Val[1];//my
				mag[2] += (float)Global_Mag_Val[2];//mz

				alt_Ref += MS5611_Get_Ref_Altitude();

				sampleTimes++;

				if(sampleTimes >= Sensors_Sample_Times)
				{
						accels[0] /= Sensors_Sample_Times;
						accels[1] /= Sensors_Sample_Times;
						accels[2] /= Sensors_Sample_Times;										 			
					
						mag[0] /= Sensors_Sample_Times;
						mag[1] /= Sensors_Sample_Times;
						mag[2] /= Sensors_Sample_Times;
						
						mag_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
						
						mag[0] /= mag_norm;
						mag[1] /= mag_norm;
						mag[2] /= mag_norm;
					
						alt_Ref /= Sensors_Sample_Times;
						Global_hgtRef = alt_Ref;						
						
						euler[0] = atan2(accels[1], accels[2]) * RAD_DEG;
						euler[1] = atan2(-1 * accels[0], accels[2]) * RAD_DEG;
					
#if defined USE_MAG
		#if defined TILT_COMPENSATION
						mag[0] *= mag_norm;
						mag[1] *= mag_norm;
						mag[2] *= mag_norm;
						euler[0] /= RAD_DEG;
						euler[1] /= RAD_DEG;
						My = ((float)mag[2] - Mag_LSM303D_Calibration.Bp_Hard_Iron_V[2]) * sin(euler[0]) - ((float)mag[1] - Mag_LSM303D_Calibration.Bp_Hard_Iron_V[1]) * cos(euler[0]);
						Mx = (((float)mag[0] - Mag_LSM303D_Calibration.Bp_Hard_Iron_V[0]) * cos(euler[1]) + ((float)mag[1] - Mag_LSM303D_Calibration.Bp_Hard_Iron_V[1]) * sin(euler[1]) * sin(euler[0]) 
									+ ((float)mag[2] - Mag_LSM303D_Calibration.Bp_Hard_Iron_V[2]) * sin(euler[1]) * cos(euler[0]));
				
						euler[2] = atan2(My, Mx) * RAD_DEG;
						Global_Heading_Ref = euler[2];
		#else
						Mx = mag[0];
						My = mag[1];
						
						#if defined USE_MAG_HMC5883L
								euler[2] = atan2(My, Mx) * RAD_DEG;
						#elif defined	USE_MAG_LSM303D
								euler[2] = atan2(-1 * My, Mx) * RAD_DEG;
						#endif						
						Global_Heading_Ref = euler[2];
						euler[2] = 0;
		#endif
#else
						euler[2] = 0;
#endif
						Euler2Quaternion(euler, q);
						
						q_norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
						
						q[0] /= q_norm;
						q[1] /= q_norm;
						q[2] /= q_norm;
						q[3] /= q_norm;
						
						Global_Init_Q[0] = q[0];
						Global_Init_Q[1] = q[1];
						Global_Init_Q[2] = q[2];
						Global_Init_Q[3] = q[3];
						
						sampleTimes = 0;
						
						alt_Ref = 0;
						
						q[0] = 1;
						q[1] = 0;
						q[2] = 0;
						q[3] = 0;
						
						accels[0] = 0;
						accels[1] = 0;
						accels[2] = 0;
						
						mag[0] = 0;
						mag[1] = 0;
						mag[2] = 0;
						
						euler[0] = 0;
						euler[1] = 0;
						euler[2] = 0;
						
						Global_HOME_Check_NONE = true;
				}
		}
}

void GPS_Dates_Deal(void)
{
#ifdef USART2_DMA_Transfer						

		if((DMA_RX_LEN2 - DMA_GetCurrDataCounter(DMA1_Stream5) == Rx_LEN) 
		&& (DMA_RX_LEN2 - DMA_GetCurrDataCounter(DMA1_Stream5) != 0))
		{							
				DMA_Cmd(DMA1_Stream5, DISABLE);
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
				GPS_Analysis(&g_gps, (uint8_t*)Global_DMA1_Stream5_Rx_Buf);								
				if(g_gps.gps_State == 1 || g_gps.gps_State == 2)// g_gps.gps_State == 1 定位状态 g_gps.gps_State == 2 差分定位
				{				
						g_gps.gps_State = 0;		
					
						if(!Global_GPS_Ref_Sample_NONE)
						{																			
								latRef += (double)g_gps.latitude / 100000 * DEG_RAD;
								lonRef += (double)g_gps.longitude / 100000 * DEG_RAD;
								altRef += (double)g_gps.altitude / 10;
							
								now_times++;
								if(now_times >= gps_Ref_Sample_times)
								{
										latRef /= gps_Ref_Sample_times;
										lonRef /= gps_Ref_Sample_times;
										altRef /= gps_Ref_Sample_times;												
									
										Global_GPS_Ref_Sample_NONE = true;	
										now_times = 0;
								}
						}
						else
						{								
								latitudeTMP = (double)g_gps.latitude / 100000 * DEG_RAD;
								longitudeTMP = (double)g_gps.longitude / 100000 * DEG_RAD;
								altitudeTMP = (float)g_gps.altitude / 10;
								
								Cal_Pos_NED(Global_GPS_Sensor.NED_Pos, latitudeTMP, longitudeTMP, altitudeTMP, latRef, lonRef, altRef);		
								Cal_Vel_NED(Global_GPS_Sensor.NED_Vel, (float)g_gps.course_earth * 0.01f, (float)g_gps.speed / 3600, 0);
							
								Global_GPS_Sensor.NED_Vel[0] = (last_Vel_NED[0] + Global_GPS_Sensor.NED_Vel[0]) * 0.5f;
								Global_GPS_Sensor.NED_Vel[1] = (last_Vel_NED[1] + Global_GPS_Sensor.NED_Vel[1]) * 0.5f;
								last_Vel_NED[0] = Global_GPS_Sensor.NED_Vel[0];
								last_Vel_NED[1] = Global_GPS_Sensor.NED_Vel[1];
								
								Global_GPS_Sensor.updated = true;
								Global_GPS_Health = true;	
								Aircraft.is_pid_update = true;
								LED2_ON; //正常状态
						}
				}
				else
				{
						Global_GPS_Health = false;
						LED2_BLINK; //GPS非健康
				}
				
				DMA_SetCurrDataCounter(DMA1_Stream5, DMA_RX_LEN2);						
				DMA_Cmd(DMA1_Stream5, ENABLE);
		}
		else
		{				
				Rx_LEN = DMA_RX_LEN2 - DMA_GetCurrDataCounter(DMA1_Stream5);
		}	
#endif
}

void Rx_eCompassVal(void)
{
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[0];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[1];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[2];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[3];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[4];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[5];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[6];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[7];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[8];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[9];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[10];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[11];
		
		Mag_LSM303D_Calibration.Bp[0][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[0][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[0][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[12];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[13];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[14];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[15];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[16];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[17];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[18];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[19];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[20];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[21];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[22];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[23];
		
		Mag_LSM303D_Calibration.Bp[1][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[1][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[1][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[24];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[25];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[26];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[27];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[28];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[29];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[30];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[31];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[32];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[33];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[34];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[35];
		
		Mag_LSM303D_Calibration.Bp[2][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[2][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[2][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
				
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[36];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[37];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[38];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[39];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[40];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[41];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[42];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[43];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[44];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[45];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[46];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[47];
		
		Mag_LSM303D_Calibration.Bp[3][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[3][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[3][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[48];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[49];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[50];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[51];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[52];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[53];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[54];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[55];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[56];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[57];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[58];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[59];
		
		Mag_LSM303D_Calibration.Bp[4][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[4][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[4][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[60];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[61];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[62];
		Flash_Mag_LSM303D_Calibration.X.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[63];
		
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[64];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[65];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[66];
		Flash_Mag_LSM303D_Calibration.Y.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[67];

		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[0] = Global_DMA1_Stream1_Rx_Buf[68];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[1] = Global_DMA1_Stream1_Rx_Buf[69];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[2] = Global_DMA1_Stream1_Rx_Buf[70];
		Flash_Mag_LSM303D_Calibration.Z.Flash_byte[3] = Global_DMA1_Stream1_Rx_Buf[71];
		
		Mag_LSM303D_Calibration.Bp[5][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		Mag_LSM303D_Calibration.Bp[5][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		Mag_LSM303D_Calibration.Bp[5][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
}

void Write_eCompassVal2Flash(uint32_t FlashAddress, Magnetometer_Calibration_Struct *SensorBpTmp)
{
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[0][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[0][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[0][2];
		Flash_wr[0] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[1] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[2] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[1][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[1][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[1][2];
		Flash_wr[3] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[4] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[5] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[2][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[2][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[2][2];
		Flash_wr[6] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[7] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[8] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
	
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[3][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[3][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[3][2];
		Flash_wr[9] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[10] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[11] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[4][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[4][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[4][2];
		Flash_wr[12] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[13] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[14] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_float = (*SensorBpTmp).Bp[5][0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_float = (*SensorBpTmp).Bp[5][1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_float = (*SensorBpTmp).Bp[5][2];
		Flash_wr[15] = Flash_Mag_LSM303D_Calibration.X.Flash_word;
		Flash_wr[16] = Flash_Mag_LSM303D_Calibration.Y.Flash_word;
		Flash_wr[17] = Flash_Mag_LSM303D_Calibration.Z.Flash_word;
		
		StmFlashWrite(FlashAddress, Flash_wr, 18);
}

void Read_Flash2eCompassVal(uint32_t FlashAddress, Magnetometer_Calibration_Struct *SensorBpTmp)
{
		StmFlashRead(FlashAddress, Flash_wr, 18);
	
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[0];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[1];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[2];
		(*SensorBpTmp).Bp[0][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[0][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[0][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
	
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[3];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[4];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[5];
		(*SensorBpTmp).Bp[1][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[1][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[1][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
	
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[6];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[7];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[8];
		(*SensorBpTmp).Bp[2][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[2][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[2][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
		
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[9];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[10];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[11];
		(*SensorBpTmp).Bp[3][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[3][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[3][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
	
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[12];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[13];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[14];
		(*SensorBpTmp).Bp[4][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[4][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[4][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;
	
		Flash_Mag_LSM303D_Calibration.X.Flash_word = Flash_wr[15];
		Flash_Mag_LSM303D_Calibration.Y.Flash_word = Flash_wr[16];
		Flash_Mag_LSM303D_Calibration.Z.Flash_word = Flash_wr[17];
		(*SensorBpTmp).Bp[5][0] = Flash_Mag_LSM303D_Calibration.X.Flash_float;
		(*SensorBpTmp).Bp[5][1] = Flash_Mag_LSM303D_Calibration.Y.Flash_float;
		(*SensorBpTmp).Bp[5][2] = Flash_Mag_LSM303D_Calibration.Z.Flash_float;	
}

void eCompass_Estimate_Effect(Magnetometer_Calibration_Struct *SensorBpTmp)
{
		(*SensorBpTmp).Bp_MatY[0] = (*SensorBpTmp).Bp[0][0] * (*SensorBpTmp).Bp[0][0] + (*SensorBpTmp).Bp[0][1] * (*SensorBpTmp).Bp[0][1] + (*SensorBpTmp).Bp[0][2] * (*SensorBpTmp).Bp[0][2];
		(*SensorBpTmp).Bp_MatY[1] = (*SensorBpTmp).Bp[1][0] * (*SensorBpTmp).Bp[1][0] + (*SensorBpTmp).Bp[1][1] * (*SensorBpTmp).Bp[1][1] + (*SensorBpTmp).Bp[1][2] * (*SensorBpTmp).Bp[1][2];
		(*SensorBpTmp).Bp_MatY[2] = (*SensorBpTmp).Bp[2][0] * (*SensorBpTmp).Bp[2][0] + (*SensorBpTmp).Bp[2][1] * (*SensorBpTmp).Bp[2][1] + (*SensorBpTmp).Bp[2][2] * (*SensorBpTmp).Bp[2][2];
		(*SensorBpTmp).Bp_MatY[3] = (*SensorBpTmp).Bp[3][0] * (*SensorBpTmp).Bp[3][0] + (*SensorBpTmp).Bp[3][1] * (*SensorBpTmp).Bp[3][1] + (*SensorBpTmp).Bp[3][2] * (*SensorBpTmp).Bp[3][2];
		(*SensorBpTmp).Bp_MatY[4] = (*SensorBpTmp).Bp[4][0] * (*SensorBpTmp).Bp[4][0] + (*SensorBpTmp).Bp[4][1] * (*SensorBpTmp).Bp[4][1] + (*SensorBpTmp).Bp[4][2] * (*SensorBpTmp).Bp[4][2];
		(*SensorBpTmp).Bp_MatY[5] = (*SensorBpTmp).Bp[5][0] * (*SensorBpTmp).Bp[5][0] + (*SensorBpTmp).Bp[5][1] * (*SensorBpTmp).Bp[5][1] + (*SensorBpTmp).Bp[5][2] * (*SensorBpTmp).Bp[5][2];		
		
		(*SensorBpTmp).Bp_MatX[0] =  (*SensorBpTmp).Bp[0][0];
		(*SensorBpTmp).Bp_MatX[1] =  (*SensorBpTmp).Bp[0][1];
		(*SensorBpTmp).Bp_MatX[2] =  (*SensorBpTmp).Bp[0][2];
		(*SensorBpTmp).Bp_MatX[3] =  1;
	
		(*SensorBpTmp).Bp_MatX[4] =  (*SensorBpTmp).Bp[1][0];
		(*SensorBpTmp).Bp_MatX[5] =  (*SensorBpTmp).Bp[1][1];
		(*SensorBpTmp).Bp_MatX[6] =  (*SensorBpTmp).Bp[1][2];
		(*SensorBpTmp).Bp_MatX[7] =  1;
	
		(*SensorBpTmp).Bp_MatX[8] =  (*SensorBpTmp).Bp[2][0];
		(*SensorBpTmp).Bp_MatX[9] =  (*SensorBpTmp).Bp[2][1];
		(*SensorBpTmp).Bp_MatX[10] =  (*SensorBpTmp).Bp[2][2];
		(*SensorBpTmp).Bp_MatX[11] =  1;
		
		(*SensorBpTmp).Bp_MatX[12] =  (*SensorBpTmp).Bp[3][0];
		(*SensorBpTmp).Bp_MatX[13] =  (*SensorBpTmp).Bp[3][1];
		(*SensorBpTmp).Bp_MatX[14] =  (*SensorBpTmp).Bp[3][2];
		(*SensorBpTmp).Bp_MatX[15] =  1;
		
		(*SensorBpTmp).Bp_MatX[16] =  (*SensorBpTmp).Bp[4][0];
		(*SensorBpTmp).Bp_MatX[17] =  (*SensorBpTmp).Bp[4][1];
		(*SensorBpTmp).Bp_MatX[18] =  (*SensorBpTmp).Bp[4][2];
		(*SensorBpTmp).Bp_MatX[19] =  1;
		
		(*SensorBpTmp).Bp_MatX[20] =  (*SensorBpTmp).Bp[5][0];
		(*SensorBpTmp).Bp_MatX[21] =  (*SensorBpTmp).Bp[5][1];
		(*SensorBpTmp).Bp_MatX[22] =  (*SensorBpTmp).Bp[5][2];
		(*SensorBpTmp).Bp_MatX[23] =  1;
		
		Matrix_Tran((*SensorBpTmp).Bp_MatX, transTMP, 6, 4);
		
		ML_R_X_ML_R(transTMP, (*SensorBpTmp).Bp_MatX, midTmp, 4, 6, 6, 4);
		
		Matrix_4X4_Inv(midTmp, invTmp);
		
		ML_R_X_ML_R(invTmp, transTMP, midTmp2, 4, 4, 4, 6);
		
		ML_R_X_ML_R(midTmp2, (*SensorBpTmp).Bp_MatY, (*SensorBpTmp).Bp_BeiTa, 4, 6, 6, 1);
		
		(*SensorBpTmp).Bp_Hard_Iron_V[0] = (*SensorBpTmp).Bp_BeiTa[0] * 0.5f;
		(*SensorBpTmp).Bp_Hard_Iron_V[1] = (*SensorBpTmp).Bp_BeiTa[1] * 0.5f;
		(*SensorBpTmp).Bp_Hard_Iron_V[2] = (*SensorBpTmp).Bp_BeiTa[2] * 0.5f;
		
		(*SensorBpTmp).Geo_B = sqrt((*SensorBpTmp).Bp_Hard_Iron_V[0] * (*SensorBpTmp).Bp_Hard_Iron_V[0] + (*SensorBpTmp).Bp_Hard_Iron_V[1] * (*SensorBpTmp).Bp_Hard_Iron_V[1] 
														  + (*SensorBpTmp).Bp_Hard_Iron_V[2] * (*SensorBpTmp).Bp_Hard_Iron_V[2] + (*SensorBpTmp).Bp_BeiTa[3]);
}

void eCompassClibration(void)
{
		if(DMA_GetCurrDataCounter(DMA1_Stream1) == 0)
		{
#ifdef USART3_DMA_Transfer
				DMA_Cmd(DMA1_Stream1, DISABLE);
				DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
				My_STM32_disable_interrupts();
				LED1_ON;
				LED2_ON;
				LED3_ON;
				LED4_ON;
				LED5_ON;
				Rx_eCompassVal();
				Write_eCompassVal2Flash(ADDR_FLASH_SECTOR_8, &Mag_LSM303D_Calibration);	
				Read_Flash2eCompassVal(ADDR_FLASH_SECTOR_8, &Mag_LSM303D_Calibration);
				eCompass_Estimate_Effect(&Mag_LSM303D_Calibration);
				Global_HOME_Check_NONE = false;
				LED1_OFF;
				LED2_OFF;
				LED3_OFF;
				LED4_OFF;
				LED5_OFF;
				My_STM32_enable_interrupts();
				DMA_SetCurrDataCounter(DMA1_Stream1, DMA_RX_LEN3);						
				DMA_Cmd(DMA1_Stream1, ENABLE);
#endif
		}
}
