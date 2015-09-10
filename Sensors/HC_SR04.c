#include "HC_SR04.h"
#include "delay.h"
#include "My_USART.h"


void My_HC_SR04_Start(void)
{
		My_USART_send_U8(Serial_1, 0X55);
}


void My_HC_SR04_Get_Height(uint16_t *hight_u)
{
		if(DMA_GetCurrDataCounter(DMA2_Stream5) == 0)
		{
#ifdef USART1_DMA_Transfer			
				DMA_Cmd(DMA2_Stream5, DISABLE);										
				DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
/*********************write your code there********************/
				*hight_u = Global_DMA2_Stream5_Rx_Buf[0] * 256 + Global_DMA2_Stream5_Rx_Buf[1];
/**************************************************************/			
				DMA_SetCurrDataCounter(DMA2_Stream5, DMA_RX_LEN1);
				DMA_Cmd(DMA2_Stream5, ENABLE);				
#endif	
		}
}
