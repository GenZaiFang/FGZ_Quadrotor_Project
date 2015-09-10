#include "MPU6050.h"
#include "My_LED.h"
#include "delay.h"
#include "My_I2C.h"

uint8_t ssdas = 14;
uint8_t isd = 9;
static uint8_t myName = 0;

void My_MPU6050_init(void)
{
    uint8_t sample_rate = 0x04; // (200Hz)(5ms)(陀螺仪输出速率1K进行5分频)
    uint8_t cfg = 0x04;         // (accel cut lf = 21Hz gyro cut lf = 20Hz)	dmp:0x02(98Hz)	匿名四轴:0x03(42Hz)
    uint8_t gyro_cfg = 0x18;    // (不自检，fs=2000deg/s)				dmp:2000 		匿名四轴:500
    uint8_t accel_cfg = 0x00;   // (不自检，fs=2g，accel cut hf = none) 		dmp:2g  		匿名四轴:4g
    uint8_t pw1_reset = 0x80, pw_1_wake_up = 0x01;// (正常启用,用x轴陀螺仪做时钟)
    uint8_t user_ctrl = 0x00;   //使用FIFO 0XC0 (不使用时取0x00)
	
		I2C_recv_str(MPU6050_SlaveAddress, MPU6050_WHO_AM_I, &myName, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, PWR_MGMT_1, &pw1_reset, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, PWR_MGMT_1, &pw_1_wake_up, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, SMPLRT_DIV, &sample_rate, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, CONFIG, &cfg, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, GYRO_CONFIG, &gyro_cfg, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, ACCEL_CONFIG, &accel_cfg, 1);
    STM32_Delay_ms(100);
    I2C_send_str(MPU6050_SlaveAddress, USER_CTRL, &user_ctrl, 1);
    STM32_Delay_ms(100);
		
		Global_Accel_Sensor.offset.x = 70;
		Global_Accel_Sensor.offset.y = -78;
		Global_Accel_Sensor.offset.z = -1469;
}

void My_MPU6050_Get_Accle_Val(int16_t *val)
{
    static uint8_t s[6], i=0;

    static union
    {
        uint8_t u8_v[2];
        int16_t i16_v;
    }accVal;

    I2C_recv_str(MPU6050_SlaveAddress, ACCEL_XOUT_H, s, 6);

    for(i=0; i<3; i++)
    {
        accVal.u8_v[1] = s[i * 2];
        accVal.u8_v[0] = s[i * 2 + 1];
        val[i] = accVal.i16_v;
    }

		val[0] *= -1;		
}

void My_MPU6050_Get_Gyro_Val(int16_t *val)
{
    static uint8_t s[6], i=0;
    static union
    {
        uint8_t u8_v[2];
        int16_t i16_v;
    }gyroVal;
		
		I2C_recv_str(MPU6050_SlaveAddress, GYRO_XOUT_H, s, 6);

    for(i=0; i<3; i++)
    {
        gyroVal.u8_v[1] = s[i * 2];
        gyroVal.u8_v[0] = s[i * 2 + 1];
        val[i] = gyroVal.i16_v;
    }

		val[1] *= -1;
		val[2] *= -1;
}

//陀螺仪校正成功返回true 不成功返回false
uint8_t Revise_MPU6050_GyroVal(void)
{
		static uint8_t enFlag = false;
		static uint16_t Sample_Times = 0;
		static double sum_Gyro[3] = {0, 0, 0};
		static double maxVal[3] = {0, 0, 0};
		static double minVal[3] = {0, 0, 0};

		if(enFlag == false)
		{
				My_MPU6050_Get_Gyro_Val(Global_Gyro_Val);

				sum_Gyro[0] += (double)Global_Gyro_Val[0];
				sum_Gyro[1] += (double)Global_Gyro_Val[1];
				sum_Gyro[2] += (double)Global_Gyro_Val[2];

				Sample_Times++;

				if(Sample_Times % Sample_Group_times == 0)
				{
						sum_Gyro[0] /= Sample_Group_times;
						sum_Gyro[1] /= Sample_Group_times;
						sum_Gyro[2] /= Sample_Group_times;

						Global_MPU6050_Gyro_Offset_Val[0] += sum_Gyro[0];
						Global_MPU6050_Gyro_Offset_Val[1] += sum_Gyro[1];
						Global_MPU6050_Gyro_Offset_Val[2] += sum_Gyro[2];

						if(Sample_Times / Sample_Group_times == 1)
						{
								maxVal[0] = sum_Gyro[0];
								minVal[0] = sum_Gyro[0];
								maxVal[1] = sum_Gyro[1];
								minVal[1] = sum_Gyro[1];
								maxVal[2] = sum_Gyro[2];
								minVal[2] = sum_Gyro[2];
						}

						if(sum_Gyro[0] > maxVal[0])
						{
								maxVal[0] = sum_Gyro[0];
						}
						else if(sum_Gyro[0] < minVal[0])
						{
								minVal[0] = sum_Gyro[0];
						}

						if(sum_Gyro[1] > maxVal[1])
						{
								maxVal[1] = sum_Gyro[1];
						}
						else if(sum_Gyro[1] < minVal[1])
						{
								minVal[1] = sum_Gyro[1];
						}

						if(sum_Gyro[2] > maxVal[2])
						{
								maxVal[2] = sum_Gyro[2];
						}
						else if(sum_Gyro[2] < minVal[2])
						{
								minVal[2] = sum_Gyro[2];
						}

						sum_Gyro[0] = 0;
						sum_Gyro[1] = 0;
						sum_Gyro[2] = 0;
				}

				if(Sample_Times >= Sample_Group * Sample_Group_times)
				{
						if((maxVal[0] - minVal[0] < MPU6050_MAX_Gyro_Error_Range)
						&& (maxVal[1] - minVal[1] < MPU6050_MAX_Gyro_Error_Range)
						&& (maxVal[2] - minVal[2] < MPU6050_MAX_Gyro_Error_Range))
						{
								Global_MPU6050_Gyro_Offset_Val[0] /= Sample_Group;
								Global_MPU6050_Gyro_Offset_Val[1] /= Sample_Group;
								Global_MPU6050_Gyro_Offset_Val[2] /= Sample_Group;

#if 1							
								Global_MPU6050_Gyro_Offset_Val[0] /= MPU6050_GYRO_DEG;
								Global_MPU6050_Gyro_Offset_Val[1] /= MPU6050_GYRO_DEG;
								Global_MPU6050_Gyro_Offset_Val[2] /= MPU6050_GYRO_DEG;
								
								Global_MPU6050_Gyro_Offset_Val[0] *= DEG_RAD;
								Global_MPU6050_Gyro_Offset_Val[1] *= DEG_RAD;
								Global_MPU6050_Gyro_Offset_Val[2] *= DEG_RAD; 
#endif							
								//LED2_ON;							
								enFlag = true;								
						}
						else
						{
								Sample_Times = 0;
								sum_Gyro[0] = 0;
								sum_Gyro[1] = 0;
								sum_Gyro[2] = 0;
								Global_MPU6050_Gyro_Offset_Val[0] = 0;
								Global_MPU6050_Gyro_Offset_Val[1] = 0;
								Global_MPU6050_Gyro_Offset_Val[2] = 0;
						}						
				}
		}
		
		return enFlag;
}
