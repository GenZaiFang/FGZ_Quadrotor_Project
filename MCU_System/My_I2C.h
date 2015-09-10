
#ifndef My_I2C_H__
#define My_I2C_H__

#include "System_Common.h"
#if 1
#define SCL_L   GPIOB->BSRRH = GPIO_Pin_10   //SCL�õ�     SCL:PB10
#define SCL_H   GPIOB->BSRRL = GPIO_Pin_10   //SCL�ø�
#define SDA_L   GPIOB->BSRRH = GPIO_Pin_11   //SDA�õ�     SDA:PB11
#define SDA_H   GPIOB->BSRRL = GPIO_Pin_11   //SDA�ø�
#define SDA_I	GPIOB->MODER &= (~(GPIO_Mode_OUT << 22)); //����SDAΪ����ģʽ�����ܣ���GPIOB->MODER�ĵ�22λ��0��
#define SDA_O   GPIOB->MODER |= GPIO_Mode_OUT << 22;      //����SDAΪ���ģʽ�����ܣ���GPIOB->MODER�ĵ�22λ��1��
#define SDA    ((GPIOB->IDR & GPIO_Pin_11) != 0) ? 1 : 0        //��ȡSDA��ֵ
#endif

void I2C_start(void);
void I2C_stop(void);
void I2C_ack(void);
void I2C_no_ack(void);
void I2C_check_ack(void);
void I2C_send_one_char(uint8_t c);
void I2C_recv_one_char(uint8_t *c);
void My_STM32_I2C_init(void);
void I2C_send_str(uint8_t slave, uint8_t reg, uint8_t *s, uint8_t num);
void I2C_recv_str(uint8_t slave, uint8_t reg, uint8_t *s, uint8_t num);

#endif
