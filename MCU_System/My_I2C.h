
#ifndef My_I2C_H__
#define My_I2C_H__

#include "System_Common.h"
#if 1
#define SCL_L   GPIOB->BSRRH = GPIO_Pin_10   //SCL置低     SCL:PB10
#define SCL_H   GPIOB->BSRRL = GPIO_Pin_10   //SCL置高
#define SDA_L   GPIOB->BSRRH = GPIO_Pin_11   //SDA置低     SDA:PB11
#define SDA_H   GPIOB->BSRRL = GPIO_Pin_11   //SDA置高
#define SDA_I	GPIOB->MODER &= (~(GPIO_Mode_OUT << 22)); //设置SDA为输入模式（功能：将GPIOB->MODER的第22位置0）
#define SDA_O   GPIOB->MODER |= GPIO_Mode_OUT << 22;      //设置SDA为输出模式（功能：将GPIOB->MODER的第22位置1）
#define SDA    ((GPIOB->IDR & GPIO_Pin_11) != 0) ? 1 : 0        //读取SDA的值
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
