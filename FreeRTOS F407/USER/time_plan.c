#include "stm32f4xx.h"
#include "time_plan.h"
#include "rtc.h"
#include "common.h"
#include "gsm.h"
#include "lat_longitude.h"
#include "norflash.h"
#include "table_process.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stdio.h"
#include <stdlib.h>
#include "sys_debug.h"
#include "electric.h"

#define UPLOAD_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 1024*1)

extern LampBuffType  LampAddr;
extern xQueueHandle  GPRSSendAddrQueue;
extern xQueueHandle  LampQueryAddrQueue;
extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];
static u8 str_run_flag;
static u32 UpTimeInvl;

void TunnelStrategyRunInit(void)
{
	TimeTypeDef time;
	
	ReadRTC_Time(RTC_Format_BIN, &time);
	
	if(time.hour < 12)
		str_run_flag = 1;
	else
		str_run_flag = 2;
	
	TunnelStrategyRun();
}

void InitUpTime(void)
{
	TimeTypeDef time;
	u32 last_time_sec,now_time_sec,up_times;
	u32 CalUpTimeMin,CalUpTimeSec;
	u16 temp[10],ManagemAddr[10]={0};
	u16 addr_7, addr1_2;
	
	NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_DATA_UPDATA_INVL_OPPSET, temp, 2);
	
	if((temp[0] == 0xFFFF) || (temp[1] == 0xFFFF))
		UpTimeInvl = 60;
	else
		UpTimeInvl = (chr2hex(temp[0])<<4) + chr2hex(temp[1]);//上传时间间隔
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, (MANAGER_ADDR_LENGTH + 1) / 2);
	addr_7 = chr2hex(ManagemAddr[1]&0x00FF);
	addr1_2 = atoi((const char *)ManagemAddr+8);
	CalUpTimeMin = addr_7*6 + ((addr1_2*7)/60)%6;
	CalUpTimeMin = CalUpTimeMin%60;
	CalUpTimeSec = (addr1_2*7)%60;
	CalUpTimeSec = CalUpTimeSec%60;
	
	ReadRTC_Time(RTC_Format_BIN, &time);
	
	now_time_sec = (u32)days(time.year, time.mon, time.day)*24*60*60 + time.hour*60*60 + time.min*60 + time.sec;
	up_times = (now_time_sec - CalUpTimeMin*60 - CalUpTimeSec)/(UpTimeInvl*60);
	last_time_sec = up_times*UpTimeInvl*60 + CalUpTimeMin*60 + CalUpTimeSec;
	
	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteBackupRegister(UP_TIME_BACKUP,last_time_sec);
}

static void vTimePlanTask(void *parameter)
{
	TimeTypeDef time;
	u32 last_time_sec,now_time_sec;
	int NetTimeResetMonth;
	u16 i;
	
	InitUpTime();
	
	for(;;)
	{
		ReadRTC_Time(RTC_Format_BIN, &time);
		
		if((str_run_flag == 2) && (time.hour < 12))//十二点之前
		{
			str_run_flag = 1;
			TunnelStrategyRun();
		}
		else if((str_run_flag == 1) && (time.hour >= 12))//十二点之后
		{
		  str_run_flag = 2;
			TunnelStrategyRun();
		}
		
		now_time_sec = (u32)days(time.year, time.mon, time.day)*24*60*60 + time.hour*60*60 + time.min*60 + time.sec;
		last_time_sec = RTC_ReadBackupRegister(UP_TIME_BACKUP);
		
		if(now_time_sec >= (last_time_sec + UpTimeInvl*60))
		{
			last_time_sec += (UpTimeInvl*60);
			PWR_BackupAccessCmd(ENABLE);
	    RTC_WriteBackupRegister(UP_TIME_BACKUP,last_time_sec);//存储上传时间
	
      if((uxQueueMessagesWaiting(GPRSSendAddrQueue) == 0) && (uxQueueMessagesWaiting(LampQueryAddrQueue) == 0))
			{
				WG_DataSample(0xFF);//网关数据查询
				for(i=0;i<LampAddr.num;i++)
				{
					xQueueSend(GPRSSendAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//单灯数据
				}
		  }
		}

		NetTimeResetMonth = RTC_ReadBackupRegister(CAL_TIME_MONTH_BACKUP);
		if((NetTimeResetMonth != time.mon) && (NetTimeResetMonth != 0))//一个月复位一次
		{
			PWR_BackupAccessCmd(ENABLE);
	    RTC_WriteBackupRegister(CAL_TIME_MONTH_BACKUP,time.mon);//更新网络校时时间备份
			
			NVIC_SystemReset();
		}
		
		vTaskDelay(configTICK_RATE_HZ);
	}
}

TaskHandle_t xTimePlanTask;

void TimePlanInit(void)
{
  xTaskCreate(vTimePlanTask, "TimePlanTask", UPLOAD_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &xTimePlanTask);
}
