#include "My_LED.h"

void My_STM32_LED_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;       		   //ͨ��IO��
    //GPIOʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  //ʹ��ʱ��PEʱ��
    //GPIO����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;//PE2��3��4��5��6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //��ʼ��Ϊ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //�˿�����50MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);                 //PE��ʼ��

    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    LED5_OFF;
}
