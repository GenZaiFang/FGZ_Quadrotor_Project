
#include "delay.h"

//-----------------------------------------------------------------
//函数名称：delay_us
//功能概要：us延时函数, 具体与晶振频率有关，延时范围：0~65535us
//          最短延时请用asm("NOP")，此指令只占用一个机器周期。
//函数返回：void
//参数说明：dat:延时长度(dat<65536)  对应总线频率：168MHz --> 168个asm(nop);指令
//-----------------------------------------------------------------
void STM32_Delay_us(uint16_t dat)
{
		while(dat--)     //先判断，再减一
		{
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
				_nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
		}
}

void STM32_Delay_ms(uint16_t dat)
{
    while(dat--)
    {
        STM32_Delay_us(1000);
    }
}

void delay_1us(void)
{
    STM32_Delay_us(1);
}

void delay_2us(void)
{
    STM32_Delay_us(2);
}

void delay_3us(void)
{
    STM32_Delay_us(3);
}

void delay_4us(void)
{
    STM32_Delay_us(4);
}

void delay_5us(void)
{
    STM32_Delay_us(5);
}
