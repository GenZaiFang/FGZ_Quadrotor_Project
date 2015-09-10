
#ifndef My_Timer_H_
#define My_Timer_H_

#include "System_Common.h"

void My_STM32_TIMER_init(void);
void Tim3_Start_Count(void);
void Tim3_End_Count(void);
void Tim1_PWM_Output(uint16_t pwm1Val, uint16_t pwm2Val, uint16_t pwm3Val, uint16_t pwm4Val);

#endif

