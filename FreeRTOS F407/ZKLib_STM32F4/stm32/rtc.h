#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_rtc.h"

#define  RTC_INIT_FLAG           RTC_BKP_DR0
#define  CENTURY_BACKUP          RTC_BKP_DR1
#define  ZONE_BACKUP             RTC_BKP_DR2
#define  UP_DATE_BACKUP          RTC_BKP_DR3
#define  UP_TIME_BACKUP          RTC_BKP_DR4
#define  CAL_TIME_MONTH_BACKUP   RTC_BKP_DR5//Ð£Ê±¸´Î»

typedef struct{
	int sec;
	int min;
	int hour;
	int day;
	int mon;
	int year;
	int week;
}TimeTypeDef;

u8 CaculateWeekDay(int y,int m, int d);
void UpdataNetTime(char *p);
u8 RtcInit(void);
void NetTimeCentury(char *p);
void ReadRTC_Time(uint32_t rtc_format, TimeTypeDef* TimeStruct);

#endif
