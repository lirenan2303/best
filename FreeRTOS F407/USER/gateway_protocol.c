#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "gsm.h"
#include "rtc.h"
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"
#include "gateway_protocol.h"
#include "ballast_protocol.h"
#include "norflash.h"
#include "common.h"
#include "lat_longitude.h"

extern TimeTypeDef time;

ErrorStatus unit_addr_check(u16 *addr, u16 *addr_hex)
{
	u16 i,j,k,l;

	i = ((*addr>>12) & 0x000f);
	j = ((*addr>>8) & 0x000f);
	k = ((*addr>>4) & 0x000f);
	l = ((*addr) & 0x000f);
	
	if(i >9 ) return ERROR;
	else if(j >9 ) return ERROR;
	else if(k >9 ) return ERROR;
	else if(l >9 ) return ERROR;
	else 
	{
		*addr_hex = i*1000+j*100+k*10+l;
	  return SUCCESS;  
	}
}

ErrorStatus Protocol_Check(u8 *buf, u8 *BufData_Size)
{
	u8 i,p_size=0;
	ErrorStatus state = ERROR;
	
	*BufData_Size = (chr2hex(*(buf+13))<<4 | chr2hex(*(buf+14)));
	
  for(i=0;i<255;i++)
	{
		p_size++;
		if(*(buf++) == 0x03)
			break;
	}
	 
	if((*BufData_Size + 18) == p_size)
	{
	  state = SUCCESS;
	}
	return state;
}

void Protocol_Response(u8 Function,u8 DataLength,u8 *databuff)
{
  u8 i,checksum;
	u8 buf[250]={0};
	u16 ManagemAddr[10]={0};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, (u16 *)&ManagemAddr, 10);

	buf[0] = 0x02;
  strncpy((char*)(buf+1), (char*)ManagemAddr, 10);

	buf[11] = hex2chr((Function>>4)&0x0f);   //������
	buf[12] = hex2chr( Function&0x0f);		
	
	buf[13] = hex2chr((DataLength>>4)&0x0f);	//�����򳤶�
  buf[14] = hex2chr( DataLength&0x0f);
	
	if(DataLength!=0)
	{
	  if(databuff!=NULL)                      //������
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
	u8 data_size=0;
	u16 WriteBuff[200]={0};
	
	if(!Protocol_Check(p, &data_size))
	{
		Protocol_Response(0xC1, 1, p+15);
		return;
	}
	
	MemStorage_Convert(p+15, data_size, WriteBuff);
	
	if(*(p+15) == 0x30) //������Ϊ������ݱ�ʶ�����ȡ�γ�ȡ�ZIGBEEƵ�㡢�Զ��ϴ�����ʱ��������·��Ƶ��ID
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA1_BASE, WriteBuff, data_size);
		rise_set();
		zigbee_config(p);
	}
	else if(*(p+15) == 0x31)  //������Ϊ����ƫ�ơ��ص�ƫ��
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA2_BASE, WriteBuff, data_size);
	}
	else if(*(p+15) == 0x32)  //������Ϊ���š������¶ȡ����ƹ�ˮλ�쳣�澯��
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
	u8 lamp_param_num,data_size=0;
	u16 WriteBuff[200]={0};
	u16 i,unit_addr,unit_addr_hex=0;
	u8 buf_temp[200],ctrl_code,n=0,j;
	

	buf_temp[0] = *(p+16);
	buf_temp[1] = *(p+17);
	buf_temp[2] = *(p+18);
	buf_temp[3] = *(p+19);
	buf_temp[4] = *(p+15);
	ctrl_code = *(p+15);
	
	if(!Protocol_Check(p, &data_size))
	{
		Protocol_Response(0xC2, 5, buf_temp);
		return;
	}
	if((data_size-1)%17 == 0)
	{
	  lamp_param_num = (data_size-1)/17;//��������
	}
	else
	{
	  Protocol_Response(0xC2, 5, buf_temp);
		return;
	}
	
	MemStorage_Convert(p+16, data_size-1, WriteBuff);
	
	if(*(p+15) == 0x31) //���Ӷ�յ��
	{
		for(i=0;i<lamp_param_num;i++)
		{
		  unit_addr =  chr2hex(WriteBuff[17*i])<<4;
		  unit_addr = (chr2hex(WriteBuff[17*i+1])+unit_addr)<<4;
		  unit_addr = (chr2hex(WriteBuff[17*i+2])+unit_addr)<<4;
		  unit_addr =  chr2hex(WriteBuff[17*i+3])+unit_addr;
			
			if(!unit_addr_check(&unit_addr, &unit_addr_hex))
				return;
			
			if(unit_addr > MAX_LAMP_NUM)
			  return;
			NorFlashWrite(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, 17);
		}
	}
	else if(*(p+15) == 0x32) //ɾ����յ��
	{
		for(i=0;i<lamp_param_num;i++)
		{
		  unit_addr =  chr2hex(WriteBuff[17*i])<<4;
		  unit_addr = (chr2hex(WriteBuff[17*i+1])+unit_addr)<<4;
		  unit_addr = (chr2hex(WriteBuff[17*i+2])+unit_addr)<<4;
		  unit_addr =  chr2hex(WriteBuff[17*i+3])+unit_addr;
			
			if(!unit_addr_check(&unit_addr, &unit_addr_hex))
				return;
			if(unit_addr > MAX_LAMP_NUM)
			  return;
			FSMC_NOR_EraseSector(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE);
		}
	}
	else if(*(p+15) == 0x33) //���Ķ�յ��
	{
	}
	else if(*(p+15) == 0x34) //ɾ�����е�
	{
		for(i=0;i<(MAX_LAMP_NUM+15)/16;i++)
		{
			FSMC_NOR_EraseBlock(NORFLASH_BALLAST_NUM + i*NORFLASH_BLOCK_SIZE);
		}
	}
	else
	{
		Protocol_Response(0xC2, 5, buf_temp);
		return;
	}
	
	for(i=0;i<lamp_param_num;i++)
	{
		for(j=0;j<4;j++)
		{
			buf_temp[n] = WriteBuff[17*i+j];
			n++;
		}
	}
	buf_temp[4*lamp_param_num] = ctrl_code;
	Protocol_Response(0x82, 4*lamp_param_num+1, buf_temp);
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

void AllParaInit(void)
{
	ReadRTC_Time(&time);
	rise_set();
}

