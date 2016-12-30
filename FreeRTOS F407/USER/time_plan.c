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
#include "sys_debug.h"

#define UPLOAD_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 1024*1)

extern LampBuffType  LampAddr;
extern xQueueHandle  GPRSSendAddrQueue;
extern xQueueHandle  GSM_GPRS_queue;
extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];


void UploadTimeBackup(void)//上传时间备份
{
	TimeTypeDef time;
	u32 up_date,up_time;
	
	ReadRTC_Time(RTC_Format_BIN, &time);
	
	up_date = time.year;
	up_date = (up_date<<8) + time.mon;
	up_date = (up_date<<8) + time.day;
	
	up_time = time.hour;
	up_time = (up_time<<8) + time.min;
	up_time = (up_time<<8) + time.sec;
	
	taskENTER_CRITICAL();//进入临界区
	
	PWR_BackupAccessCmd(ENABLE);
	RTC_WriteBackupRegister(UP_DATE_BACKUP,up_date);
	RTC_WriteBackupRegister(UP_TIME_BACKUP,up_time);
	
	taskEXIT_CRITICAL();//退出临界区
}

static u8 str_run_flag;

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

static void vTimePlanTask(void *parameter)
{
	TimeTypeDef time,upload_time;
	u32 last_upload_time_min,now_time_min,time_invl;
	u16 i,temp[10];
	u8 WG_query[]= {0x02,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x30,0x38,0x30,0x39,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x33,0x42,0x03};
	int NetTimeResetMonth;
	
	if((RTC_ReadBackupRegister(UP_DATE_BACKUP) == 0) && (RTC_ReadBackupRegister(UP_TIME_BACKUP) == 0))
	{
    UploadTimeBackup();
	}
		
	for(;;)
	{
		NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_DATA_UPDATA_INVL_OPPSET, temp, 2);
		
		if((temp[0] == 0xFFFF) || (temp[1] == 0xFFFF))
			time_invl = 60;
		else
			time_invl = (chr2hex(temp[0])<<4) + chr2hex(temp[1]);//上传时间间隔
		
		ReadRTC_Time(RTC_Format_BIN, &time);
		ReadUpload_Time(RTC_Format_BIN, &upload_time);
		
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
		
		now_time_min = days(time.year, time.mon, time.day)*24*60 + time.hour*60 + time.min;
		last_upload_time_min = days(upload_time.year, upload_time.mon, upload_time.day)*24*60 + upload_time.hour*60 + upload_time.min;
		if((now_time_min - last_upload_time_min) >= time_invl)
		{
			if(uxQueueMessagesWaiting(GPRSSendAddrQueue) == 0)
			{
			  xQueueSend(GSM_GPRS_queue, &WG_query, configTICK_RATE_HZ*5);//网关数据查询
	
				for(i=0;i<LampAddr.num;i++)
				{
				  xQueueSend(GPRSSendAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//单灯数据
				}
			  UploadTimeBackup();//存储上传时间
			}
		}

		NetTimeResetMonth = RTC_ReadBackupRegister(RESET_TIME_BACKUP);
		if((NetTimeResetMonth != time.mon) && (NetTimeResetMonth != 0))//一个月复位一次
		{
			PWR_BackupAccessCmd(ENABLE);
			RTC_WriteBackupRegister(RESET_TIME_BACKUP,time.mon);
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
