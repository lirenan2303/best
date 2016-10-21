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
#include "norflash.h"
#include "common.h"
#include "lat_longitude.h"

void Protocol_Response(u8 Function,u8 DataLength,u8 *databuff)
{
  u8 i,checksum;
	u8 buf[250]={0};
	u16 ManagemAddr[10]={0};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, (u16 *)&ManagemAddr, 10);

	buf[0] = 0x02;
  strncpy((char*)(buf+1), (char*)ManagemAddr, 10);

	buf[11] = hex2chr((Function>>4)&0x0f);   //功能码
	buf[12] = hex2chr( Function&0x0f);		
	
	buf[13] = hex2chr((DataLength>>4)&0x0f);	//数据域长度
  buf[14] = hex2chr( DataLength&0x0f);
	
	if(DataLength!=0)
	{
	  if(databuff!=NULL)                                //数据域
		{
		  for(i=0;i<DataLength;i++)
	      buf[i+15] = *(databuff+i);  
		}                                        			
	}
	
	checksum = BCC_CheckSum(buf,DataLength+15);
	buf[DataLength+15] = hex2chr((checksum>>4)&0x0f);
  buf[DataLength+16] = hex2chr( checksum&0x0f    );
	buf[DataLength+17] = 0x03;

	SimSendData(buf, DataLength+18);
}

void zigbee_config(u8 *buf)
{
	
}

void HandleGatewayParam(u8 *p)
{
	u8 i,p_size=0;
	u8 *buf,data_size=0;
	u16 WriteBuff[200]={0};
	
	
	buf = p;
	data_size = (chr2hex(*(buf+13))<<4 | chr2hex(*(buf+14)));
	
  for(i=0;i<100;i++)
	{
		p_size++;
		if(*(buf++) == 0x03)
			break;
	}
	 
	if((data_size + 18) != p_size)
	{
		Protocol_Response(0xC1, 1, p+15);
		return;
	}
	
	MemStorage_Convert(p+15, data_size, WriteBuff);
	
	if(*(p+15) == 0x30) //数据域为网关身份标识、经度、纬度、ZIGBEE频点、自动上传数据时间间隔、回路、频点ID
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA1_BASE, WriteBuff, data_size);
		rise_set();
		zigbee_config(p);
	}
	else if(*(p+15) == 0x31)  //数据域为开灯偏移、关灯偏移
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA2_BASE, WriteBuff, data_size);
	}
	else if(*(p+15) == 0x32)  //数据域为柜门、网关温度、控制柜水位异常告警打开
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA3_BASE, WriteBuff, data_size);
	}
	else
	{
	  Protocol_Response(0xC1, 1, p+15);
		return;
	}
	
	Protocol_Response(0x81, 1, p+15);	
}

void HandleLightParam(u8 *p)
{
	if(*(p+15) == 0x30) //增加一盏灯
	{
		
	}
	else if(*(p+15) == 0x31) //删除一盏灯
	{
		
	}
	else if(*(p+15) == 0x32) //更改一盏灯参数
	{
		
	}
	else if(*(p+15) == 0x33) //删除所有灯
	{
		
	}
}

void HandleStrategyDownload(u8 *p)
{
	
}

void HandleLightDimmer(u8 *p)
{
	
}

void HandleLightOnOff(u8 *p)
{
	
}

