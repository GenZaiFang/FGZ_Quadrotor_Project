
#ifndef My_USART_H_
#define My_USART_H_

#include "System_Common.h"

//´®¿ÚºÅ 1..3
#define Serial_1               1
#define Serial_2               2
#define Serial_3               3


void My_USARTn_DMA_Config(void);
void My_STM32_USART_init(uint32_t *baudrate, uint8_t *INT_EN_FLAG);
void My_USART_send_U8(uint8_t USARTn, uint8_t val);
void My_USART_send_MUX_Bytes_x(uint8_t USARTn, uint8_t x_flag, float val);
uint8_t My_USART_read_U8(uint8_t USARTn);

#endif
