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

u16 LampQueryAddrTable[MAX_LAMP_NUM] = {0};//需要查询的单灯地址列表�
u16 GSM_SendAddrTable[MAX_LAMP_NUM] = {0};//GSM需要发送的单灯地址列表
u8 LampDataTempBuff[MAX_LAMP_NUM][LAMP_DATA_TEMP_SIZE]  __attribute__((at(CCMDATARAM_BASE))) = {0};//读镇流器数据存储数组
LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff)))) = {0};//镇流器段属性地址排序
LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM]  __attribute__((at(CCMDATARAM_BASE + sizeof(LampDataTempBuff) + sizeof(LampAttrSortTable)))) = {0};//镇流器运行状态及控制数组

void LampSortFunction(void)
{
	u16 Branch_ID,Segment_ID,Pole_ID;
	u16 address;
	
	
	
	
}
void AllTableInit(void)
{
	
	
}
