#include "My_Flash.h"

void My_STM32_Flash_init(void)
{
	FLASH_SetLatency(FLASH_Latency_5); // fHCLK = 168MHz need set 5 wait states
}

uint32_t StmGetSectorEndAddress(uint32_t Address, uint32_t *EndAddress)
{
	uint32_t sector = 0;
	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_Sector_0;
		*EndAddress = ADDR_FLASH_SECTOR_1 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_Sector_1;
		*EndAddress = ADDR_FLASH_SECTOR_2 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_Sector_2;
		*EndAddress = ADDR_FLASH_SECTOR_3 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_Sector_3;
		*EndAddress = ADDR_FLASH_SECTOR_4 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_Sector_4;
		*EndAddress = ADDR_FLASH_SECTOR_5 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_Sector_5;
		*EndAddress = ADDR_FLASH_SECTOR_6 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_Sector_6;
		*EndAddress = ADDR_FLASH_SECTOR_7 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_Sector_7;
		*EndAddress = ADDR_FLASH_SECTOR_8 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_Sector_8;
		*EndAddress = ADDR_FLASH_SECTOR_9 - 1;
	}
		else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_Sector_9;
		*EndAddress = ADDR_FLASH_SECTOR_10 - 1;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_Sector_10;
		*EndAddress = ADDR_FLASH_SECTOR_11 - 1;
	}
	else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
	{
		sector = FLASH_Sector_11;
		*EndAddress = 0x080FFFFF;
	}
	return sector;
}

void StmFlashWrite(uint32_t FlashAddress, uint32_t *WriteDatasBuf, uint32_t Nums)
{	
	uint32_t iNums = 0;
	uint32_t endAddress = ADDR_FLASH_SECTOR_1 - 1;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
							 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);	
	FLASH_EraseSector(StmGetSectorEndAddress(FlashAddress, &endAddress), VoltageRange_3);
	
	if(Nums != 0)
	{
		for(iNums = 0; iNums < Nums; iNums++)
		{
			if(FlashAddress < endAddress)
			{
				while(FLASH_ProgramWord(FlashAddress, WriteDatasBuf[iNums]) != FLASH_COMPLETE);
				FlashAddress += 4; //地址移动4个字节 32位
			}
		}
	}
	FLASH_Lock();
}

void StmFlashRead(uint32_t FlashAddress, uint32_t *ReadDatasBuf, uint32_t Nums)
{
	uint32_t iNums = 0;
	uint32_t endAddress = ADDR_FLASH_SECTOR_1 - 1;
	StmGetSectorEndAddress(FlashAddress, &endAddress);
	if(Nums != 0)
	{		
		for(iNums = 0; iNums < Nums; iNums++)
		{
			if(FlashAddress < endAddress)
			{				
				ReadDatasBuf[iNums] = *(__IO uint32_t*)FlashAddress;
				FlashAddress += 4; //地址移动4个字节 32位
			}
		}
	}
}
