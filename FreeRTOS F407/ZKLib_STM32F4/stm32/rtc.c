#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "rtc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_gpio.h"
#include "lat_longitude.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "delay.h"
#include "common.h"
#include "time_plan.h"

SemaphoreHandle_t RTC_SystemRunningSemaphore;
NVIC_InitTypeDef   NVIC_InitStructure;

void RTC_BackUpCheck(void)
{
	u32 temp[2]={0};
	char Zone_re[4] = "+8\r";
	
	if(RTC_ReadBackupRegister(CENTURY_BACKUP) != 20)
	{
		PWR_BackupAccessCmd(ENABLE);	
		RTC_WriteBackupRegister(CENTURY_BACKUP,20);	//���ͱ���
	}
	
	temp[0] = RTC_ReadBackupRegister(ZONE_BACKUP);
	if(strstr((char*)temp, "\r") == NULL)
	{
		strcpy((char*)temp, Zone_re);
		PWR_BackupAccessCmd(ENABLE);	
		RTC_WriteBackupRegister(ZONE_BACKUP,temp[0]);	//ʱ������
	}
}


u8 RtcInit(void)
{
  RTC_InitTypeDef RTC_InitStructure;
	
	RTC_SystemRunningSemaphore = xSemaphoreCreateMutex();
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//ʹ��PWRʱ��
	PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ������� 
	
	if(RTC_ReadBackupRegister(RTC_INIT_FLAG)!=0xA5A5)		//�Ƿ��һ������? 0xA5A5Ϊ��Ϊ�����Ƿ����ñ�׼
	{
		RCC_LSEConfig(RCC_LSE_ON);//LSE ����
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);	//���ָ����RCC��־λ�������,�ȴ����پ������
			
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//����RTCʱ��(RTCCLK),ѡ��LSE��ΪRTCʱ��    
		RCC_RTCCLKCmd(ENABLE);	//ʹ��RTCʱ�� 

    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;//RTC�첽��Ƶϵ��(1~0X7F)
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;//RTCͬ����Ƶϵ��(0~7FFF)
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;//RTC����Ϊ,24Сʱ��ʽ
    RTC_Init(&RTC_InitStructure);
	 
		RTC_WriteBackupRegister(RTC_INIT_FLAG,0xA5A5);	//����Ѿ���ʼ������
	}
	
	RTC_BackUpCheck();
	
	return true;
}

u8 CaculateWeekDay(int y,int m, int d) //��ķ����ɭ��ʽ
{
	int Week;
	
  if((m==1) || (m==2)) 
  {
    m+=12;
		y--;
  }
	
  Week=(d+2*m+3*(m+1)/5+y+y/4-y/100+y/400)%7;
	
  switch(Week)
  {
    case 0: return 1;
    case 1: return 2; 
    case 2: return 3; 
    case 3: return 4;
    case 4: return 5;
    case 5: return 6; 
    case 6: return 7;//������
		default:return 0;
  }
}

void UpdataNetTime(char *p)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
  RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	char buf[30];
	short temp[10];
//	int Zone_value = 0;
	
	sscanf(p, "%*s%s", buf); 
	
  if(sscanf(buf,"\"%hd/%hd/%hd,%hd:%hd:%hd+%hd\"",&temp[0],&temp[1],&temp[2],&temp[3],&temp[4], &temp[5], &temp[6]) != 7)//���Уʱ�ַ�����ʽ�Ƿ���ȷ
    return;
	
//	Zone_value = temp[6];
//	if(RTC_ReadBackupRegister(ZONE_BACKUP) != Zone_value)
//	{
//	  PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ������� 
//	  RTC_WriteBackupRegister(ZONE_BACKUP,Zone_value);	//ʱ������
//	}

	RTC_DateTypeInitStructure.RTC_Year = Bcd2ToByte(chr2hex(buf[1])<<4 | chr2hex(buf[2]));
	RTC_DateTypeInitStructure.RTC_Month = Bcd2ToByte(chr2hex(buf[4])<<4 | chr2hex(buf[5]));
	RTC_DateTypeInitStructure.RTC_Date = Bcd2ToByte(chr2hex(buf[7])<<4 | chr2hex(buf[8]));
  RTC_TimeTypeInitStructure.RTC_Hours = Bcd2ToByte(chr2hex(buf[10])<<4 | chr2hex(buf[11]));
	RTC_TimeTypeInitStructure.RTC_Minutes = Bcd2ToByte(chr2hex(buf[13])<<4 | chr2hex(buf[14]));
	RTC_TimeTypeInitStructure.RTC_Seconds = Bcd2ToByte(chr2hex(buf[16])<<4 | chr2hex(buf[17]));
	
	RTC_DateTypeInitStructure.RTC_WeekDay = CaculateWeekDay(((u16)RTC_DateTypeInitStructure.RTC_Year + RTC_ReadBackupRegister(RTC_BKP_DR1)*100), 
	                                        RTC_DateTypeInitStructure.RTC_Month, RTC_DateTypeInitStructure.RTC_Date);
	
	if(RTC_DateTypeInitStructure.RTC_Year < 16)//����Ƿ�С��16��
		return;
	
	if(RTC_DateTypeInitStructure.RTC_WeekDay == 0)
	  return;
	
	if(xSemaphoreTake(RTC_SystemRunningSemaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
	  RTC_SetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
		RTC_SetDate(RTC_Format_BIN,&RTC_DateTypeInitStructure);
		
		vTaskDelay(1);
		xSemaphoreGive(RTC_SystemRunningSemaphore);
	}
}

void NetTimeCentury(char *p)
{
	char buf[30];
	short temp[10];
	int YearCentury = 0;
	
	sscanf(p, "%*s%s", buf); 
	
  if(sscanf(buf,"%hd,%hd,%hd,%hd,%hd,%hd,",&temp[0],&temp[1],&temp[2],&temp[3],&temp[4], &temp[5]) == 6)//���Уʱ�ַ�����ʽ�Ƿ���ȷ
	{
	  YearCentury = Bcd2ToByte(chr2hex(buf[0])<<4 | chr2hex(buf[1]));
		
		if(RTC_ReadBackupRegister(CENTURY_BACKUP) != YearCentury)
		{
			PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ������� 
			RTC_WriteBackupRegister(CENTURY_BACKUP,YearCentury);	//���ͱ���
		}
	}
}

void ReadRTC_Time(uint32_t rtc_format, TimeTypeDef* TimeStruct)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
  RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	if(xSemaphoreTake(RTC_SystemRunningSemaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
		RTC_GetDate(rtc_format,&RTC_DateTypeInitStructure);
		RTC_GetTime(rtc_format,&RTC_TimeTypeInitStructure);
		
		TimeStruct->sec = RTC_TimeTypeInitStructure.RTC_Seconds;
	  TimeStruct->min = RTC_TimeTypeInitStructure.RTC_Minutes;
	  TimeStruct->hour = RTC_TimeTypeInitStructure.RTC_Hours;
	  TimeStruct->day = RTC_DateTypeInitStructure.RTC_Date;
	  TimeStruct->mon = RTC_DateTypeInitStructure.RTC_Month;
		if(rtc_format == RTC_Format_BIN)
	    TimeStruct->year = RTC_DateTypeInitStructure.RTC_Year + RTC_ReadBackupRegister(CENTURY_BACKUP)*100;
		else if(rtc_format == RTC_Format_BCD)
		  TimeStruct->year = RTC_DateTypeInitStructure.RTC_Year + (ByteToBcd2(RTC_ReadBackupRegister(CENTURY_BACKUP))<<8);
	  TimeStruct->week = RTC_DateTypeInitStructure.RTC_WeekDay;
		
		xSemaphoreGive(RTC_SystemRunningSemaphore);
	}
}
