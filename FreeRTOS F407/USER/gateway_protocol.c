#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "gsm.h"
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"
#include "gateway_protocol.h"


/*******************************************************************************
* ��������  : BCC_CheckSum
* ��    ��  : ���У��
* �������  : buf��У���׵�ַ
              len��У���ֽ��� 
* ��    ��  : 16����У���
*******************************************************************************/
u8 BCC_CheckSum(u8 *buf, u8 len)
{
  unsigned char  i;
  unsigned char checksum = *buf;

  for(i = 0; i < len-1; i++) 
  {
    checksum ^= *(++buf);
  }
  return checksum;
}
/*******************************************************************************
* ��������  : chr2hex
* ��    ��  : ASCII��ת��Ϊ���ֽ�16������
* �������  : chr 
* ��    ��  : ���ֽ�16������
*******************************************************************************/
u8 chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9') return (chr-'0');
	if(chr>='A'&&chr<='F') return (chr-'A'+10);
	if(chr>='a'&&chr<='f') return (chr-'a'+10); 
	return (0);
}
/*******************************************************************************
* ��������  : hex2chr
* ��    ��  : ���ֽ�16������ת��ΪASCII��
* �������  : hex 
* ��    ��  : ASCII��
*******************************************************************************/
u8 hex2chr(u8 hex)
{
	if(hex<=9)            return (hex+'0');
	if(hex>=10&&hex<=15)  return (hex-10+'A'); 
	return ('0');
}

void HandleGatewayParam(u8 *p)
{
	
}
