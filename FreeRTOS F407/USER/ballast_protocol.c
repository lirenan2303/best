#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "ballast_comm.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "gsm.h"
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"
#include "norflash.h"
#include "common.h"
#include "ballast_protocol.h"

//u8 AttrCount[8][6] = {0};
u16 LampQueryAddrTable[MAX_LAMP_NUM]={0};//ÐèÒª²éÑ¯µÄµ¥µÆµØÖ·ÁÐ±íí
u16 GSM_SendAddrTable[MAX_LAMP_NUM]={0};//GSMÐèÒª·¢ËÍµÄµ¥µÆµØÖ·ÁÐ±í
u8 LampDataTempBuff[MAX_LAMP_NUM][LAMP_DATA_TEMP_SIZE]  __attribute__((at(CCMDATARAM_BASE)));//¶ÁÕòÁ÷Æ÷Êý¾Ý´æ´¢Êý×é
LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff))));//ÕòÁ÷Æ÷¶ÎÊôÐÔµØÖ·ÅÅÐò
LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff) + sizeof(LampAttrSortTable))));//ÕòÁ÷Æ÷ÔËÐÐ×´Ì¬¼°¿ØÖÆÊý×é

u16 lamp_num=0;

//memset(LampAttrSortTable, 0, sizeof(LampAttrSortTable));
//memset(LampRunCtrlTable, 0, sizeof(LampRunCtrlTable));
//memset(LampDataTempBuff, 0, sizeof(MAX_LAMP_NUM*LAMP_DATA_TEMP_SIZE));

void branch_quick_sort(int addr_offset, int left, int right)//»ØÂ·ÅÅÐò
{
	if(left >= right)
	{
		return;
	}
	int i = left;
	int j = right;
	LampAttrSortType key = LampAttrSortTable[addr_offset + left];
	
	while(i < j)                             
	{
		while(i < j && key.branch <= LampAttrSortTable[addr_offset + j].branch)
		{
			j--;
		}
		LampAttrSortTable[addr_offset + i] = LampAttrSortTable[addr_offset + j];
		 
		while(i < j && key.branch >= LampAttrSortTable[addr_offset + i].branch)
		{
			i++;
		}
		LampAttrSortTable[addr_offset + j] = LampAttrSortTable[addr_offset + i];
	}
	
	LampAttrSortTable[addr_offset + i] = key;
	
  branch_quick_sort(addr_offset, left, i - 1);
	branch_quick_sort(addr_offset, i + 1, right); 
}

void segment_quick_sort(int addr_offset, int left, int right)//¶ÎÅÅÐò
{
	if(left >= right)
	{
		return;
	}
	int i = left;
	int j = right;
	LampAttrSortType key = LampAttrSortTable[addr_offset + left];
	
	while(i < j)                             
	{
		while(i < j && key.segment <= LampAttrSortTable[addr_offset + j].segment)
		{
			j--;
		}
		LampAttrSortTable[addr_offset + i] = LampAttrSortTable[addr_offset + j];
		 
		while(i < j && key.segment >= LampAttrSortTable[addr_offset + i].segment)
		{
			i++;
		}
		LampAttrSortTable[addr_offset + j] = LampAttrSortTable[addr_offset + i];
	}
	
	LampAttrSortTable[addr_offset + i] = key;
	
  segment_quick_sort(addr_offset, left, i - 1);
	segment_quick_sort(addr_offset, i + 1, right);
}

void pole_quick_sort(int addr_offset, int left, int right)//»ØÂ·ÅÅÐò
{
	if(left >= right)
	{
		return;
	}
	int i = left;
	int j = right;
	LampAttrSortType key = LampAttrSortTable[addr_offset + left];
	
	while(i < j)                             
	{
		while(i < j && key.pole_id <= LampAttrSortTable[addr_offset + j].pole_id)
		{
			j--;
		}
		LampAttrSortTable[addr_offset + i] = LampAttrSortTable[addr_offset + j];
		 
		while(i < j && key.pole_id >= LampAttrSortTable[addr_offset + i].pole_id)
		{
			i++;
		}
		LampAttrSortTable[addr_offset + j] = LampAttrSortTable[addr_offset + i];
	}
	
	LampAttrSortTable[addr_offset + i]= key;
	
  pole_quick_sort(addr_offset, left, i - 1);
	pole_quick_sort(addr_offset, i + 1, right); 
}

void LampAttrSort(void)
{
	u16 m;	
	u16 index,num_temp,index_base;
	u8 branch_temp,segment_temp;
	
	index = 0;
	num_temp = 0;
	index_base = 0;
  branch_quick_sort(index + index_base, 0, lamp_num-1);//»ØÂ·ÅÅÐò
	
	index = 0;
	num_temp = 0;
	index_base = 0;
	while(1)//¶ÎÅÅÐò
	{
		branch_temp = LampAttrSortTable[index].branch;
		index_base += num_temp;
		num_temp = 0;
		
		if(index >= lamp_num)
				break;
		for(m=0;m<MAX_LAMP_NUM;m++)
		{
			index++;
			num_temp++;
			if(index >= lamp_num)
				break;
			if(LampAttrSortTable[index].branch != branch_temp)
				break;
		}
		segment_quick_sort(index_base, 0, num_temp-1);
  }
	
	index = 0;
	num_temp = 0;
	index_base = 0;
	while(1)//µÆ¸ËºÅÅÅÐò
	{
		branch_temp = LampAttrSortTable[index].branch;
		segment_temp = LampAttrSortTable[index].segment;
		index_base += num_temp;
		num_temp = 0;
		
		if(index >= lamp_num)
				break;
		for(m=0;m<MAX_LAMP_NUM;m++)
		{
			index++;
			num_temp++;
			if(index >= lamp_num)
				break;
			if((LampAttrSortTable[index].branch != branch_temp) || (LampAttrSortTable[index].segment != segment_temp))
				break;
		}
		pole_quick_sort(index_base, 0, num_temp-1);
	}
}

void LampSortFunction(void)
{
	u16 i,lamp_index=0;
	u16 WriteBuff[20];
  u8 branch_temp,segment_temp;
	u16 polo_temp,unit_addr;
	LampAttrSortType LampAttrTemp;
	
	for(i=0;i<MAX_LAMP_NUM;i++)
	{
		NorFlashRead(NORFLASH_BALLAST_BASE + i*NORFLASH_SECTOR_SIZE, (u16 *)&WriteBuff, 17);
		
	  if(WriteBuff[0] == 0xffff)
			continue;
		
	  unit_addr =  chr2hex(WriteBuff[0])<<4;
	  unit_addr = (chr2hex(WriteBuff[1])+unit_addr)<<4;
		unit_addr = (chr2hex(WriteBuff[2])+unit_addr)<<4;
		unit_addr =  chr2hex(WriteBuff[3])+unit_addr;
		
		branch_temp = chr2hex(WriteBuff[8]);
		
		polo_temp =  chr2hex(WriteBuff[9])<<4;
	  polo_temp = (chr2hex(WriteBuff[10])+polo_temp)<<4;
		polo_temp = (chr2hex(WriteBuff[11])+polo_temp)<<4;
		polo_temp =  chr2hex(WriteBuff[12])+polo_temp;
		
		segment_temp =  chr2hex(WriteBuff[15])<<4;
	  segment_temp = (chr2hex(WriteBuff[16])+segment_temp);
		
		LampAttrTemp.branch = branch_temp;
		LampAttrTemp.segment = segment_temp;
		LampAttrTemp.pole_id = polo_temp;
		LampAttrTemp.addr = unit_addr;
		
		LampAttrSortTable[lamp_index++] = LampAttrTemp;
		lamp_num++;
	}
	LampAttrSort();
}

void AllTableInit(void)
{
	memset(LampDataTempBuff, 0, sizeof(LampDataTempBuff));
  memset(LampAttrSortTable, 0, sizeof(LampAttrSortTable));
  memset(LampRunCtrlTable, 0, sizeof(LampRunCtrlTable));
	
	taskENTER_CRITICAL();//½øÈëÁÙ½çÇø£¨ÏµÍ³¿ÉÆÁ±ÎÖÐ¶Ï£©
	LampSortFunction();
	taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
}

void HandleUnitLightParam(u8 *p)
{
	
}

void HandleUnitStrategy(u8 *p)
{
	
}

void HandleUnitReadData(u8 *p)
{
	
}
