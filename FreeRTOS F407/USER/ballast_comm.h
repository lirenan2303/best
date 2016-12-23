#ifndef __ZIGBEE_COMM_H
#define __ZIGBEE_COMM_H

#include "stm32f4xx.h"

#define  MAX_QUERY_NUM    3

void BallastCommInit(void);
void UnitComm1DMA_TxBuff(u8 *buf, u8 buf_size);

#endif
