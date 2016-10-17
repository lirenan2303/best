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
#include "ballast_protocol.h"

u16 LampQueryAddrTable[MAX_LAMP_NUM] = {0};//ÐèÒª²éÑ¯µÄµ¥µÆµØÖ·ÁÐ±íí
u16 GSM_SendAddrTable[MAX_LAMP_NUM] = {0};//GSMÐèÒª·¢ËÍµÄµ¥µÆµØÖ·ÁÐ±í
u8 LampDataTempBuff[MAX_LAMP_NUM][LAMP_DATA_TEMP_SIZE]  __attribute__((at(CCMDATARAM_BASE))) = {0};
LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff)))) = {0};
LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff) + sizeof(LampAttrSortTable)))) = {0};


void AllTableInit(void)
{
	
}
