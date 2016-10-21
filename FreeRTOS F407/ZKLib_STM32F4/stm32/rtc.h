#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_rtc.h"

typedef struct{
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
}TimeTypeDef;

void UpdataNetTime(char *p);
u8 RtcInit(void);
void NetTimeCentury(char *p);
void ReadRTC_Time(TimeTypeDef* TimeStruct);

#endif
