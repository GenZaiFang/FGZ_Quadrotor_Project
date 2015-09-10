#ifndef My_Flash_h_
#define My_Flash_h_

#include "System_Common.h"

void My_STM32_Flash_init(void);
void StmFlashWrite(uint32_t FlashAddress, uint32_t *WriteDatasBuf, uint32_t Nums);
void StmFlashRead(uint32_t FlashAddress, uint32_t *ReadDatasBuf, uint32_t Nums);

#endif
