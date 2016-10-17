#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_rtc.h"

void UpdataNetTime(char *p);
u8 RtcInit(void);
void NetTimeCentury(char *p);

#endif
