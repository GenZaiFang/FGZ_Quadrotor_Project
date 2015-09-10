
#include "HMC5883L.h"
#include "My_USART.h"
#include "My_I2C.h"
#include "delay.h"

//-----------------------------------------------------------------
//�������ƣ�hmc5883l_init
//���ܸ�Ҫ��HMC5883L��ʼ��    
//�������أ�void
//����˵������
//-----------------------------------------------------------------
void MY_HMC5883L_init(void)
{
		static uint8_t a_cfg = 0x74;			//����ƽ����8�������������30Hz��������������
		static uint8_t b_cfg = 0x20;			//���棺1090
		static uint8_t mod_cfg = 0x00;		//ģʽ���ã���������ģʽ
		
		I2C_send_str(HMC5883L_SlaveAddress, A_CFG, &a_cfg, 1);
		I2C_send_str(HMC5883L_SlaveAddress, B_CFG, &b_cfg, 1);
		I2C_send_str(HMC5883L_SlaveAddress, MODE_CFG, &mod_cfg, 1);
}

//-----------------------------------------------------------------
//�������ƣ�hmc5883l_read_mag
//���ܸ�Ҫ����ȡHMC5883������ų�ǿ��ֵ���������׵�ַΪdat��3���洢��Ԫ
//�������أ�void
//����˵����dat :��ȡ��3���ų�ǿ��ֵ��ŵ��׵�ַ
//-----------------------------------------------------------------
void MY_HMC5883L_Get_mag_Val(int16_t *dat) 
{
    uint8_t s[6];
    union
		{
				int16_t i16_Val;
				uint8_t u8_Val[2];
		}magVal;

    I2C_recv_str(HMC5883L_SlaveAddress, XOUT_H, s, 6); 
    
    magVal.u8_Val[1] = s[0];
    magVal.u8_Val[0] = s[1];
    *(dat + 0) = magVal.i16_Val - (478 - 467) / 2; //magnetic->x
    
    magVal.u8_Val[1] = s[2];
    magVal.u8_Val[0] = s[3];
    *(dat + 2) = magVal.i16_Val - (140 - 533) / 2;//magnetic->z
    
    magVal.u8_Val[1] = s[4];
    magVal.u8_Val[0] = s[5];
    *(dat + 1) = magVal.i16_Val - (307 - 328) / 2; //magnetic->y
}

void My_HMC5883L_Mag_Check(void)
{
static int16_t maxVal[3] = {0, 0, 0};
static int16_t minVal[3] = {0, 0, 0};		
static int16_t magTMP[3] = {1, 0, 0};		

		MY_HMC5883L_Get_mag_Val(magTMP);

		maxVal[0] = magTMP[0];
		maxVal[1] = magTMP[1];
		maxVal[2] = magTMP[2];

		minVal[0] = magTMP[0];
		minVal[1] = magTMP[1];
		minVal[2] = magTMP[2];

		STM32_Delay_ms(8);

		for(;;)
		{
				MY_HMC5883L_Get_mag_Val(magTMP);					
				
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
#if 1
				My_USART_send_MUX_Bytes_x(Serial_3, 0x2f, maxVal[1]);
				My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, minVal[1]);
				My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, magTMP[1]);			
#endif						
				STM32_Delay_ms(8);
		}
}

float MY_HMC5883L_Get_angle(void)
{
		static int16_t tem[3];   		              //���������ֵ
		static float dat = 0;		
		int16_t ssd[2];
		
		MY_HMC5883L_Get_mag_Val(tem);                   //��ȡ���������ֵ
	
		ssd[0] = tem[0];// - (1620 + 660) * 0.5; // x
		ssd[1] = tem[1];// - (-220 - 840) * 0.5; // y

#if 0
		if(tem[0] > 0x7fff)
		{
				tem[0] -= 0xffff;	                    //X
		}
		
		if(tem[2] > 0x7fff)
		{
				tem[2] -= 0xffff;	                    //Y
		}
#endif
		dat = atan2((float)ssd[1], (float)ssd[0]) * (180 / 3.14159265); 
		
		return dat;										
}

