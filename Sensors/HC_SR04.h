#ifndef HC_SR04_H_
#define HC_SR04_H_

#include "System_Common.h"

#define Trig_DOWN   GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define Trig_UP     GPIO_SetBits(GPIOC, GPIO_Pin_6)

void My_HC_SR04_Start(void);
void My_HC_SR04_Get_Height(uint16_t *hight_u);

#endif
