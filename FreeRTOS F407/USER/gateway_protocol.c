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
* 函数名称  : BCC_CheckSum
* 描    述  : 异或校验
* 输入参数  : buf：校验首地址
              len：校验字节数 
* 返    回  : 16进制校验和
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
* 函数名称  : chr2hex
* 描    述  : ASCII码转换为单字节16进制数
* 输入参数  : chr 
* 返    回  : 单字节16进制数
*******************************************************************************/
u8 chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9') return (chr-'0');
	if(chr>='A'&&chr<='F') return (chr-'A'+10);
	if(chr>='a'&&chr<='f') return (chr-'a'+10); 
	return (0);
}
/*******************************************************************************
* 函数名称  : hex2chr
* 描    述  : 单字节16进制数转换为ASCII码
* 输入参数  : hex 
* 返    回  : ASCII码
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
