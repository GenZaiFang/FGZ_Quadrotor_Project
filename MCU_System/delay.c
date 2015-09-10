
#include "delay.h"

//-----------------------------------------------------------------
//�������ƣ�delay_us
//���ܸ�Ҫ��us��ʱ����, �����뾧��Ƶ���йأ���ʱ��Χ��0~65535us
//          �����ʱ����asm("NOP")����ָ��ֻռ��һ���������ڡ�
//�������أ�void
//����˵����dat:��ʱ����(dat<65536)  ��Ӧ����Ƶ�ʣ�168MHz --> 168��asm(nop);ָ��
//-----------------------------------------------------------------
void STM32_Delay_us(uint16_t dat)
{
		while(dat--)     //���жϣ��ټ�һ
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
