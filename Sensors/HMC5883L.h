
#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "System_Common.h"

//����HMC5883L�ڲ���ַ��ָ��
#define	HMC5883L_SlaveAddress     0x3C	   //�ӻ���ַ
#define A_CFG    0x00                      //(W/R)���üĴ���A�����ò���ƽ����������������ʣ��������ã�Ĭ��ֵ��11110000
#define B_CFG    0x01                      //(W/R)���üĴ���B���������棻Ĭ��ֵ��00100000
#define MODE_CFG 0x02                      //(W/R)ģʽ�Ĵ��������ò���ģʽ����������һ��Ĭ��ֵ��00000001
#define XOUT_H   0x03                      //(R)��������Ĵ���X��8λ
#define XOUT_L   0x04                      //(R)��������Ĵ���X��8λ
#define ZOUT_H   0x05
#define ZOUT_L   0x06
#define YOUT_H   0x07
#define YOUT_L   0x08
#define STATUS   0x09                      //(R)״̬�Ĵ���
#define IDR_A    0x010                     //(R)ʶ��Ĵ���A
#define IDR_B    0x011                     //(R)ʶ��Ĵ���B
#define IDR_C    0x012                     //(R)ʶ��Ĵ���C

//-----------------------------------------------------------------
//�������ƣ�hmc5883l_init
//���ܸ�Ҫ��HMC5883L��ʼ��    
//�������أ�void
//����˵������
//-----------------------------------------------------------------
void MY_HMC5883L_init(void);

//-----------------------------------------------------------------
//�������ƣ�hmc5883l_read_mag
//���ܸ�Ҫ����ȡHMC5883������ų�ǿ��ֵ���������׵�ַΪdat��3���洢��Ԫ
//�������أ�void
//����˵����dat :��ȡ��3���ų�ǿ��ֵ��ŵ��׵�ַ
//-----------------------------------------------------------------
void MY_HMC5883L_Get_mag_Val(int16_t *dat);


void My_HMC5883L_Mag_Check(void);

float MY_HMC5883L_Get_angle(void);


#endif
