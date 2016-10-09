#ifndef __UNICODE2GBK_H__
#define __UNICODE2GBK_H__

#include <stdint.h>
#include "FreeRTOS.h"

/// \brief  ��һ��UCS2������ַ�ת����GBK������ַ�.
/// \param  unicode   ��Ҫת����UCS2�����ַ���.
/// \param  len       ��Ҫת����UCS2�����ַ������ֽ���(���ַ���).
/// \return !=NULL    ת���õ�GBK�ַ���,��0x00����,ʹ��֮�������Unicode2GBKDestroy�ͷ��ڴ�.
/// \return ==NULL    ת��ʱ�����ڴ�ʧ��.
uint8_t *Unicode2GBK(const uint8_t *unicode, int len);

/// \brief  �ͷ���UCS2ת���õ�GBK�ַ���.
/// \param  p   ��Ҫ�ͷŵ�GBK�ַ���.
static inline void Unicode2GBKDestroy(uint8_t *p) {
	vPortFree(p);
}

#endif
