#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"
#include "semphr.h"
#include "norflash.h"
#include "sys_debug.h"

static SemaphoreHandle_t norflash_semaphore;

void NorFlashInit(void) 
{
	if( norflash_semaphore == NULL )
	{
    FSMC_NOR_Init();
	  norflash_semaphore = xSemaphoreCreateMutex();
	}
}

void NorFlashWrite(u32 addr, const u16 *ram, int len)
{
	NOR_Status status = NOR_ONGOING;
  if (xSemaphoreTake(norflash_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		status = FSMC_NOR_EraseSector(addr);
		if(status != NOR_SUCCESS)
		{
			 printf_str("\r\nNOR_FLASH EraseSector Error!!!");
		}
		status = FSMC_NOR_WriteBuffer(ram, addr, len);
		if(status != NOR_SUCCESS)
		{
		   printf_str("\r\nNOR_FLASH Write Error!!!");
		}
		xSemaphoreGive(norflash_semaphore);
  }
}

void NorFlashEraseSector(uint32_t addr)
{
	NOR_Status status = NOR_ONGOING;
	
  if (xSemaphoreTake(norflash_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		status = FSMC_NOR_EraseSector(addr);
		if(status != NOR_SUCCESS)
		{
			 printf_str("\r\nNOR_FLASH Erase Sector Error!!!");
		}
		xSemaphoreGive(norflash_semaphore);
  }
}

void NorFlashEraseBlock(uint32_t addr)
{
	NOR_Status status = NOR_ONGOING;
	
  if (xSemaphoreTake(norflash_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		status = FSMC_NOR_EraseBlock(addr);
		if(status != NOR_SUCCESS)
		{
			 printf_str("\r\nNOR_FLASH Erase Block Error!!!");
		}
		xSemaphoreGive(norflash_semaphore);
  }
}

void NorFlashEraseChip(void) 
{
	NOR_Status status = NOR_ONGOING;
	
  if (xSemaphoreTake(norflash_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		status = FSMC_NOR_EraseChip();
		if(status != NOR_SUCCESS)
			 printf_str("\r\nNOR_FLASH Erase Chip Error!!!");
		xSemaphoreGive(norflash_semaphore);
  }
}

void NorFlashRead(u32 addr, u16 *ram, int len) 
{
  if (xSemaphoreTake(norflash_semaphore, configTICK_RATE_HZ * 5) == pdTRUE)
  {
    FSMC_NOR_ReadBuffer(ram, addr, len);
		xSemaphoreGive(norflash_semaphore);
  }
}

void MemStorage_Convert(u8 *msg, u16 cvt_size, u16 *write_buf)
{
  u8 i;
	
	for(i=0;i<cvt_size;i++)
	{
		*(write_buf+i) = *(msg+i);
	}
}

