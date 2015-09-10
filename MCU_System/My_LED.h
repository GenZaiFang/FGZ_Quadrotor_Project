
#ifndef My_LED_H_
#define My_LED_H_

#include "System_Common.h"
void My_STM32_LED_init(void);

#define LED1_ON   GPIO_ResetBits(GPIOE, GPIO_Pin_2)
#define LED2_ON   GPIO_ResetBits(GPIOE, GPIO_Pin_3)
#define LED3_ON   GPIO_ResetBits(GPIOE, GPIO_Pin_4)
#define LED4_ON   GPIO_ResetBits(GPIOE, GPIO_Pin_5)
#define LED5_ON   GPIO_ResetBits(GPIOE, GPIO_Pin_6)

#define LED1_OFF    GPIO_SetBits(GPIOE, GPIO_Pin_2)
#define LED2_OFF    GPIO_SetBits(GPIOE, GPIO_Pin_3)
#define LED3_OFF    GPIO_SetBits(GPIOE, GPIO_Pin_4)
#define LED4_OFF    GPIO_SetBits(GPIOE, GPIO_Pin_5)
#define LED5_OFF    GPIO_SetBits(GPIOE, GPIO_Pin_6)

#define LED1_BLINK   GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)(!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_2)))
#define LED2_BLINK   GPIO_WriteBit(GPIOE, GPIO_Pin_3, (BitAction)(!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_3)))
#define LED3_BLINK   GPIO_WriteBit(GPIOE, GPIO_Pin_4, (BitAction)(!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_4)))
#define LED4_BLINK   GPIO_WriteBit(GPIOE, GPIO_Pin_5, (BitAction)(!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_5)))
#define LED5_BLINK   GPIO_WriteBit(GPIOE, GPIO_Pin_6, (BitAction)(!GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_6)))

#endif
