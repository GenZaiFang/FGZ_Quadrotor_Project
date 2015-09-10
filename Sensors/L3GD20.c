#include "L3GD20.h"
#include "My_USART.h"
#include "delay.h"
#include "My_I2C.h"
#include "My_LED.h"

static uint8_t myName = 0;

//L3GD20初始化
void My_L3GD20_init(void)
{
    uint8_t ctrl_reg1 = 0xCF; //11 00 1 111  ODR=760Hz,CUT_OFF=30Hz,normal_mode,XYZ axis enable
    uint8_t ctrl_reg2 = 0x00; //00 00 0000   mormal_mode
    uint8_t ctrl_reg3 = 0x00; //0 0 0 0 0 0 0 0 INT_disable
    uint8_t ctrl_reg4 = 0x30; //0 0 11 0 00 0 连续更新，量程：2000°/s,SPI四线模式
    uint8_t ctrl_reg5 = 0x00; //0 0 0 0 00 00 FIFO_disable,HPF_disable,

		I2C_recv_str(L3GD20_SlaveAddress, L3GD20_WHO_AM_I, &myName, 1);
    STM32_Delay_ms(10);
    I2C_send_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG1, &ctrl_reg1, 1);
    STM32_Delay_ms(10);
    I2C_send_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG2, &ctrl_reg2, 1);
    STM32_Delay_ms(10);
    I2C_send_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG3, &ctrl_reg3, 1);
    STM32_Delay_ms(10);
    I2C_send_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG4, &ctrl_reg4, 1);
    STM32_Delay_ms(10);
    I2C_send_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG5, &ctrl_reg5, 1);
    STM32_Delay_ms(10);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_CTRL_REG1, &ctrl_reg5, 1);
    STM32_Delay_ms(10);
}

//读取L3GD20陀螺仪数据
void My_L3GD20_Get_gyro(int16_t *dat)
{
    static uint8_t s[6], i = 0;
    static union
    {
        uint8_t u8_v[2];
        int16_t i16_v;
    }gyroVal;

    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_X_L, s  , 1);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_X_H, s+1, 1);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_Y_L, s+2, 1);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_Y_H, s+3, 1);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_Z_L, s+4, 1);
    I2C_recv_str(L3GD20_SlaveAddress, L3GD20_OUT_Z_H, s+5, 1); //寄存器地址没有自增功能

    for(i = 0; i < 3; i++)
    {
    	gyroVal.u8_v[0] = s[i * 2];
    	gyroVal.u8_v[1] = s[i * 2 + 1];
    	dat[i] = gyroVal.i16_v;
    }
}

uint8_t Revise_L3GD20_GyroVal(void)
{
		static uint8_t enFlag = false;
		static uint16_t Sample_Times = 0;
		static double sum_Gyro[3] = {0, 0, 0};
		static double maxVal[3] = {0, 0, 0};
		static double minVal[3] = {0, 0, 0};

		if(enFlag == false)
		{
				My_L3GD20_Get_gyro(Global_Gyro_Val);

				sum_Gyro[0] += (double)Global_Gyro_Val[0];
				sum_Gyro[1] += (double)Global_Gyro_Val[1];
				sum_Gyro[2] += (double)Global_Gyro_Val[2];

				Sample_Times++;

				if(Sample_Times % Sample_Group_times == 0)
				{
						sum_Gyro[0] /= Sample_Group_times;
						sum_Gyro[1] /= Sample_Group_times;
						sum_Gyro[2] /= Sample_Group_times;

						Global_L3GD20_Gyro_Offset_Val[0] += sum_Gyro[0];
						Global_L3GD20_Gyro_Offset_Val[1] += sum_Gyro[1];
						Global_L3GD20_Gyro_Offset_Val[2] += sum_Gyro[2];

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
						if((maxVal[0] - minVal[0] < L3GD20_MAX_Gyro_Error_Range)
						&& (maxVal[1] - minVal[1] < L3GD20_MAX_Gyro_Error_Range)
						&& (maxVal[2] - minVal[2] < L3GD20_MAX_Gyro_Error_Range))
						{
								Global_L3GD20_Gyro_Offset_Val[0] /= Sample_Group;
								Global_L3GD20_Gyro_Offset_Val[1] /= Sample_Group;
								Global_L3GD20_Gyro_Offset_Val[2] /= Sample_Group;		
								LED1_ON;							
								enFlag = true;								
						}
						else
						{
								Sample_Times = 0;
								sum_Gyro[0] = 0;
								sum_Gyro[1] = 0;
								sum_Gyro[2] = 0;
								Global_L3GD20_Gyro_Offset_Val[0] = 0;
								Global_L3GD20_Gyro_Offset_Val[1] = 0;
								Global_L3GD20_Gyro_Offset_Val[2] = 0;
						}						
				}
		}		
		return enFlag;
}
