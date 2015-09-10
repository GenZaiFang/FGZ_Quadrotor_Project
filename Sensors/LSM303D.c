#include "LSM303D.h"
#include "My_USART.h"
#include "delay.h"
#include "My_I2C.h"
#include "CommonPlan.h"
#include "My_Flash.h"
#include "My_LED.h"

static uint8_t myName = 0;
static uint32_t init_write_flash = 0;

//LSM303D初始化
void My_LSM303D_init(void)
{
    uint8_t ctrl0 = 0x04;  //0 0 0 00 1 00  High-pass filter enabled
    uint8_t ctrl1 = 0x97;  //1001 0 111     AODR= 800 Hz; continuous update; ZYX-axis enabled
    uint8_t ctrl2 = 0x03;  //00 000 0 1 1;  773Hz;±2g;Acceleration self-test enable;SPI Serial Interface mode:3-wire interface
    uint8_t ctrl3 = 0x00;  //00000000       INT1 disable
    uint8_t ctrl4 = 0x00;  //00000000       INT2 disable 
    uint8_t ctrl5 = 0xF4;  //1 11 101 00    temperature sensor enabled;Magnetic resolution high;MODR=100Hz;interrupt request not latched
    uint8_t ctrl6 = 0x40;  //0 10 00000     Magnetic full-scale:±8gauss
    uint8_t ctrl7 = 0x00;  //00 0 0 0 0 00  High-pass filter mode:Normal mode;internal filter bypassed;Temperature sensor only mode:0;Magnetic sensor mode:Continuous-conversion mode
     
		I2C_recv_str(LSM303D_SlaveAddress, LSM303D_WHO_AM_I, &myName, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL0, &ctrl0, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL1, &ctrl1, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL2, &ctrl2, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL3, &ctrl3, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL4, &ctrl4, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL5, &ctrl5, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL6, &ctrl6, 1);
		STM32_Delay_ms(10);
		I2C_send_str(LSM303D_SlaveAddress, LSM303D_CTRL7, &ctrl7, 1);
		STM32_Delay_ms(10);
		
		StmFlashRead(ADDR_FLASH_SECTOR_9, &init_write_flash, 1);
		
		if(init_write_flash == 0xFFFFFFFF) // 只会在第一次下载代码时候运行一次
		{
				init_write_flash = 0x12345678;
				StmFlashWrite(ADDR_FLASH_SECTOR_9, &init_write_flash, 1);
			
				Mag_LSM303D_Calibration.Bp[0][0] = 1034;
				Mag_LSM303D_Calibration.Bp[0][1] = -520;
				Mag_LSM303D_Calibration.Bp[0][2] = -275;
				
				Mag_LSM303D_Calibration.Bp[1][0] = 145;
				Mag_LSM303D_Calibration.Bp[1][1] = 798;
				Mag_LSM303D_Calibration.Bp[1][2] = -474;
				
				Mag_LSM303D_Calibration.Bp[2][0] = -1462;
				Mag_LSM303D_Calibration.Bp[2][1] = 33;
				Mag_LSM303D_Calibration.Bp[2][2] = -455;
				
				Mag_LSM303D_Calibration.Bp[3][0] = -90;
				Mag_LSM303D_Calibration.Bp[3][1] = -1973;
				Mag_LSM303D_Calibration.Bp[3][2] = 129;
				
				Mag_LSM303D_Calibration.Bp[4][0] = 817;
				Mag_LSM303D_Calibration.Bp[4][1] = -1024;
				Mag_LSM303D_Calibration.Bp[4][2] = -472;
				
				Mag_LSM303D_Calibration.Bp[5][0] = -1193;
				Mag_LSM303D_Calibration.Bp[5][1] = -1135;
				Mag_LSM303D_Calibration.Bp[5][2] = -10;
				
				Write_eCompassVal2Flash(ADDR_FLASH_SECTOR_8, &Mag_LSM303D_Calibration);
				
				LED1_ON;
				LED2_ON;
				LED3_ON;
				LED4_ON;
				LED5_ON;
				
				for(;;);				
		}
		
		Read_Flash2eCompassVal(ADDR_FLASH_SECTOR_8, &Mag_LSM303D_Calibration);
		
		eCompass_Estimate_Effect(&Mag_LSM303D_Calibration);
#if 1	
		Mag_LSM303D_Calibration.inv_M[0] = 1.0f;
		Mag_LSM303D_Calibration.inv_M[1] = 0.0f;
		Mag_LSM303D_Calibration.inv_M[2] = 0.0f;
		
		Mag_LSM303D_Calibration.inv_M[3] = 0.0f;
		Mag_LSM303D_Calibration.inv_M[4] = 1.0f;
		Mag_LSM303D_Calibration.inv_M[5] = 0.0f;
		
		Mag_LSM303D_Calibration.inv_M[6] = 0.0f;
		Mag_LSM303D_Calibration.inv_M[7] = 0.0f;
		Mag_LSM303D_Calibration.inv_M[8] = 1.0f;
#else
		Mag_LSM303D_Calibration.inv_M[0] = 0.9990625871533f;
		Mag_LSM303D_Calibration.inv_M[1] = 0.004583751863281f;
		Mag_LSM303D_Calibration.inv_M[2] = -0.000496783217153f;
		
		Mag_LSM303D_Calibration.inv_M[3] = 0.004583751863281f;
		Mag_LSM303D_Calibration.inv_M[4] = 0.968625096545439f;
		Mag_LSM303D_Calibration.inv_M[5] = 0.002815959079813f;
		
		Mag_LSM303D_Calibration.inv_M[6] = -0.000496783217153f;
		Mag_LSM303D_Calibration.inv_M[7] = 0.002815959079813f;
		Mag_LSM303D_Calibration.inv_M[8] = 1.02281747193783f;
#endif
}

//读取LSM303D加速度计数据
void My_LSM303D_Get_Accle_Val(int16_t *Val)
{
    static uint8_t s[6], i = 0;    
		union
		{
				int16_t i16_Val;
				uint8_t u8_Val[2];
		}accVal;

    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_X_L_A, s + 0, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_X_H_A, s + 1, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Y_L_A, s + 2, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Y_H_A, s + 3, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Z_L_A, s + 4, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Z_H_A, s + 5, 1); //寄存器地址没有自增功能 

    for(i=0; i<3; i++)
    {
				accVal.u8_Val[0] = s[i*2];
				accVal.u8_Val[1] = s[i*2+1];
				Val[i] = accVal.i16_Val;
    }
		Val[0] *= -1;
}

//读取LSM303D磁力计数据
void My_LSM303D_Get_Mag_Val(int16_t *Val)
{
    static uint8_t s[6], i=0;

    union
		{
				int16_t i16_Val;
				uint8_t u8_Val[2];
		}magVal;

    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_X_L_M, s + 0, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_X_H_M, s + 1, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Y_L_M, s + 2, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Y_H_M, s + 3, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Z_L_M, s + 4, 1); 
    I2C_recv_str(LSM303D_SlaveAddress, LSM303D_OUT_Z_H_M, s + 5, 1); //寄存器地址没有自增功能
    
    for(i = 0; i < 3; i++)
    {
				magVal.u8_Val[0] = s[i * 2];
				magVal.u8_Val[1] = s[i * 2 + 1];
				Val[i] = magVal.i16_Val;
    }
		Val[1] *= -1; 
		Val[2] *= -1;
}

void My_LSM303D_Mag_Check(void)
{
		static int16_t maxVal[3] = {0, 0, 0};
		static int16_t minVal[3] = {0, 0, 0};
		
		int16_t magTMP[3];
	
		My_LSM303D_Get_Mag_Val(magTMP);
		maxVal[0] = magTMP[0];
		maxVal[1] = magTMP[1];
		maxVal[2] = magTMP[2];
		minVal[0] = magTMP[0];
		minVal[1] = magTMP[1];
		minVal[2] = magTMP[2];
		STM32_Delay_ms(8);

		for(;;)
		{
				My_LSM303D_Get_Mag_Val(magTMP);		
				
				if(magTMP[0] > maxVal[0])
				{
						maxVal[0] = magTMP[0];
				}
				else if(magTMP[0] < minVal[0])
				{
						minVal[0] = magTMP[0];
				}
				
				if(magTMP[1] > maxVal[1])
				{
						maxVal[1] = magTMP[1];
				}
				else if(magTMP[1] < minVal[1])
				{
						minVal[1] = magTMP[1];
				}
				
				if(magTMP[2] > maxVal[2])
				{
						maxVal[2] = magTMP[2];
				}
				else if(magTMP[2] < minVal[2])
				{
						minVal[2] = magTMP[2];
				}
										
				My_USART_send_MUX_Bytes_x(Serial_3, 0x1f, magTMP[0]);
				My_USART_send_MUX_Bytes_x(Serial_3, 0x2f, magTMP[1]);				
				My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, magTMP[2]);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, minVal[1]);						
				STM32_Delay_ms(8);
		}
}

float My_LSM303D_Get_yaw(int16_t *magTMP)
{		
		float heading = 0;
		float norm;
		float mx1, my1, mz1;		
	
		mx1 = (float)magTMP[0] - (-1260 + 720) * 0.5;
		my1 = (float)magTMP[1] - (-1400 + 550) * 0.5;
		mz1 = (float)magTMP[2] - (1300 - 760) * 0.5;
		norm = sqrt(mx1 * mx1 + my1 * my1 + mz1 * mz1);
		mx1 /= norm;
		my1 /= norm;
		mz1 /= norm;
		
		//mx2 = mx1 * cos(Global_pitch) + mz1 * sin(Global_pitch);
		//my2 = mx1 * sin(Global_roll) * sin(Global_pitch) + my1 * cos(Global_roll) - mz1 * sin(Global_roll) * cos(Global_pitch);
	
		heading = atan2(my1, mx1) * RAD_DEG;
		#if 0
		if(mx2 > 0 && my2 >=0)
		{
				heading = atan2(my2, mx2) * RAD_DEG;
		}
		else if(mx2 < 0)
		{
				heading = 180 + atan2(my2, mx2) * RAD_DEG;
		}
		else if(mx2 > 0 && my2 <=0)
		{
				heading = 360 + atan2(my2, mx2) * RAD_DEG;
		}
		else if(mx2 == 0 && my2 < 0)
		{
				heading = 90;
		}
		else if(mx2 == 0 && my2 > 0)
		{
				heading = 270;
		}
		#endif
		return heading;
}
