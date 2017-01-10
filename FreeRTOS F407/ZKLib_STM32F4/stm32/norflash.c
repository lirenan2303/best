#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"
#include "semphr.h"
#include "norflash.h"
#include "sys_debug.h"
#include "gsm.h"
#include "norflash.h"
#include <string.h>

static SemaphoreHandle_t norflash_semaphore;

void NorFlashDataInit(void)
{
	u16 readbuff[15]={0};
	u16 buff[50]={0};
	WG_ServerParameterType   DebugWG_ServerPara;
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, readbuff, (MANAGER_ADDR_LENGTH + 1) / 2);
	if(NorflashDataCheck(readbuff, (MANAGER_ADDR_LENGTH + 1) / 2) == EMPTY)
	{
		strcpy((char*)buff, "9999999999");
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, buff, (MANAGER_ADDR_LENGTH + 1) / 2);
	}
	
	memset(buff, 0, sizeof(buff));
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, readbuff, (sizeof(DebugWG_ServerPara) + 1)/ 2);	
	if(NorflashDataCheck(readbuff, (sizeof(DebugWG_ServerPara) + 1)/2) == EMPTY)
	{
		strcpy((char*)buff, "61.190.38.46");
		strcpy((char*)buff+16, "30001");	
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, buff, sizeof(buff)/2);
	}
}

void NorFlashInit(void)
{
  FSMC_NOR_Init();

	norflash_semaphore = xSemaphoreCreateMutex();
	
	NorFlashDataInit();
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

void ConvertToByte(u16 *write_buf, u16 cvt_size, u8 *msg)
{
  u8 i;
	
	for(i=0;i<cvt_size;i++)
	{
	  *(msg+i) = *(write_buf+i);
	}
}

DataStateTypeDef NorflashDataCheck(u16*p, u16 size)
{
	u16 i;
	
	for(i=0;i<size;i++)
	{
		if(*(p+i) == 0xFFFF)
			return EMPTY;
	}
	return NO_EMPTY;
}
