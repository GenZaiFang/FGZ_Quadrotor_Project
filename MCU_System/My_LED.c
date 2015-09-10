#include "My_LED.h"

void My_STM32_LED_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;       		   //通用IO口
    //GPIO时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  //使能时钟PE时钟
    //GPIO配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;//PE2、3、4、5、6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //初始化为输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);                 //PE初始化

    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    LED5_OFF;
}
