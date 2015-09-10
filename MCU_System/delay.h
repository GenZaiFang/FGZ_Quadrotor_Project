
#ifndef DELAY_H_
#define DELAY_H_

#include "System_Common.h"

#define _nop_() __nop()

void STM32_Delay_us(uint16_t dat);
void STM32_Delay_ms(uint16_t dat);

void delay_1us(void);
void delay_2us(void);
void delay_3us(void);
void delay_4us(void);
void delay_5us(void);

#endif
