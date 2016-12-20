#ifndef __ZIGBEE_COMM_H
#define __ZIGBEE_COMM_H

#include "stm32f4xx.h"

void BallastCommInit(void);
void UnitComm1DMA_TxBuff(u8 *buf, u8 buf_size);

#endif
