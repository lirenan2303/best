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
#include "table_process.h"
#include "norflash.h"
#include "common.h"
#include "lat_longitude.h"

extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];
extern TimeTypeDef time;
extern u16 lamp_num;

void RTC_TimeToChar(u16 *buf)
{              
	ReadRTC_Time(RTC_Format_BCD, &time);

	*buf++ = hex2chr((time.tm_year>>4) & 0x0000000f);
	*buf++ = hex2chr(time.tm_year & 0x0000000f);
	*buf++ = hex2chr((time.tm_mon>>4) & 0x0000000f);
	*buf++ = hex2chr(time.tm_mon & 0x0000000f);
	*buf++ = hex2chr((time.tm_mday>>4) & 0x0000000f);
	*buf++ = hex2chr(time.tm_mday & 0x0000000f);
	*buf++ = hex2chr((time.tm_hour>>4) & 0x0000000f);
	*buf++ = hex2chr(time.tm_hour & 0x0000000f);
	*buf++ = hex2chr((time.tm_min>>4) & 0x0000000f);
	*buf++ = hex2chr(time.tm_min & 0x0000000f);
	*buf++ = hex2chr((time.tm_sec>>4) & 0x0000000f);
	*buf   = hex2chr(time.tm_sec & 0x0000000f);
}

ErrorStatus unit_addr_check(u16 addr, u16 *addr_hex)
{
	u16 i,j,k,l;

	i = ((addr>>12) & 0x000f);
	j = ((addr>>8) & 0x000f);
	k = ((addr>>4) & 0x000f);
	l = ((addr) & 0x000f);
	
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

ErrorStatus GatewayAddrCheck(u8 *addr_buf)
{
	u16 ManagemAddr[10]={0};
	u8  BroadcastAddr[]={0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, 10);
	
	if(strncmp((char*)addr_buf, (char*)ManagemAddr , 10) == 0)
		return SUCCESS;
  else if(strncmp((char*)addr_buf, (char*)BroadcastAddr , 10) == 0)
		return SUCCESS;
	else
		return ERROR;
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

void Protocol_Response(u8 Function, u8 *databuff, u8 DataLength)
{
  u8 i,checksum;
	u8 buf[250]={0};
	u16 ManagemAddr[10]={0};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, 10);

	buf[0] = 0x02;
  strncpy((char*)(buf+1), (char*)ManagemAddr, 10);

	buf[11] = hex2chr((Function>>4)&0x0f);   //功能码
	buf[12] = hex2chr( Function&0x0f);		
	
	buf[13] = hex2chr((DataLength>>4)&0x0f);	//数据域长度
  buf[14] = hex2chr( DataLength&0x0f);
	
	if(DataLength!=0)
	{
	  if(databuff!=NULL)                      //数据域
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

void zigbee_config(void)
{
}

void HandleGatewayParam(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[200]={0};
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
		return;
	
	MemStorage_Convert(p+15, data_size, WriteBuff);
	
	if(*(p+15) == 0x30) //数据域为网关身份标识、经度、纬度、ZIGBEE频点、自动上传数据时间间隔、回路、频点ID
	{
		NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA1_BASE, WriteBuff, data_size);
		rise_set();
		zigbee_config();
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
	  Protocol_Response(0xC1, p+15, 1);
		return;
	}
	
	Protocol_Response(0x81, p+15, 1);	
}

void HandleLampParam(u8 *p)
{
	u8 lamp_param_num,data_size=0;
	u16 WriteBuff[50]={0};
	u16 unit_addr_bcd,unit_addr_hex=0;
	u8 buf_temp[200],n=0,j;
	int i;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
		return;
	
	if((data_size-1)%17 == 0)
	{
	  lamp_param_num = (data_size-1)/17;//参数数量
	}
	else
		return;

	if(*(p+15) == 0x31) //增加多盏灯
	{
		for(i=0;i<lamp_param_num;i++)
		{
			WriteBuff[0] = *(p+16+17*i);//地址
			WriteBuff[1] = *(p+16+17*i+1);
			WriteBuff[2] = *(p+16+17*i+2);
			WriteBuff[3] = *(p+16+17*i+3);
			
			unit_addr_bcd =  chr2hex(WriteBuff[0])<<4;
		  unit_addr_bcd = (chr2hex(WriteBuff[1])+unit_addr_bcd)<<4;
		  unit_addr_bcd = (chr2hex(WriteBuff[2])+unit_addr_bcd)<<4;
		  unit_addr_bcd =  chr2hex(WriteBuff[3])+unit_addr_bcd;
			
			if(!unit_addr_check(unit_addr_bcd, &unit_addr_hex))
				return;
			if(unit_addr_hex > MAX_LAMP_NUM)
			  return;
			
	  	RTC_TimeToChar(WriteBuff+4);//灯参数同步标识
			
			for(j=0;j<13;j++)//相关数据
			{
				WriteBuff[16+j] = *(p+20+17*i+j);
			}
			NorFlashWrite(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, 29);
		}
	}
	else if(*(p+15) == 0x32) //删除多盏灯
	{
		for(i=0;i<lamp_param_num;i++)
		{
		  WriteBuff[0] = *(p+16+17*i);//地址
			WriteBuff[1] = *(p+16+17*i+1);
			WriteBuff[2] = *(p+16+17*i+2);
			WriteBuff[3] = *(p+16+17*i+3);
			
			unit_addr_bcd =  chr2hex(WriteBuff[0])<<4;
		  unit_addr_bcd = (chr2hex(WriteBuff[1])+unit_addr_bcd)<<4;
		  unit_addr_bcd = (chr2hex(WriteBuff[2])+unit_addr_bcd)<<4;
		  unit_addr_bcd =  chr2hex(WriteBuff[3])+unit_addr_bcd;
			
			if(!unit_addr_check(unit_addr_bcd, &unit_addr_hex))
				return;
			if(unit_addr_hex > MAX_LAMP_NUM)
			  return;
			
			NorFlashEraseSector(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE);
		}
	}
	else if(*(p+15) == 0x33) //更改多盏灯
	{
	}
	else if(*(p+15) == 0x34) //删除所有灯
	{
		for(i=0;i<(MAX_LAMP_NUM+15)/16;i++)
		{
			NorFlashEraseBlock(NORFLASH_BALLAST_NUM + i*NORFLASH_BLOCK_SIZE);
		}
	}
	else
	{
		Protocol_Response(0xC2, buf_temp, 5);
		return;
	}
	
	for(i=0;i<lamp_param_num;i++)
	{
		for(j=0;j<4;j++)
		{
			buf_temp[n] = *(p+16+17*i+j);
			n++;
		}
	}
	buf_temp[4*lamp_param_num] = *(p+15);
	Protocol_Response(0x82, buf_temp, 4*lamp_param_num+1);
}

void HandleLampStrategy(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[200]={0};
	u16 i,unit_addr_hex;
	u8 buf_temp[10];
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
		return;
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))
	{
		RTC_TimeToChar(WriteBuff+29);//灯参数同步标识
		MemStorage_Convert(p+23, data_size-8, WriteBuff+29+12);
		
		for(i=0;i<lamp_num;i++)
		{
			unit_addr_check(LampAttrSortTable[i].addr, &unit_addr_hex);
			
			NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, 29);//读出灯参数
			NorFlashWrite(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, 29+12+data_size-8);//写入灯参数和策略
		}
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 10);
	
	Protocol_Response(0x83, buf_temp, 10);
}

void HandleLampDimmer(u8 *p)
{
	u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
    return;
	
}

void unit_turn_on(u8 branch, u8 data)
{
	
}

void unit_turn_off(u8 branch, u8 data)
{
	
}

void HandleLampOnOff(u8 *p)
{
	u8 data_size=0;
	u8 branch_num,segment1_num,segment2_num;
	u8 i,j;
	u8 buf_temp[30];
	u16 unit_addr_bcd;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
    return;
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//网关开关灯
	{
		if(*(p+19) == 0x30)
			unit_turn_on(ALL_BRANCH, ALL_SEGMENT);
		else if(*(p+19) == 0x31)
			unit_turn_off(ALL_BRANCH, ALL_SEGMENT);
	}
	else if(*(p+15) == 0x38)//回路属性开关灯
	{
		branch_num = chr2hex(*(p+16));
		segment1_num = chr2hex(*(p+17));
		segment2_num = chr2hex(*(p+18));
		for(i=0;i<branch_num;i++)
		{
			for(j=0;j<segment1_num;j++)
			{
				if(*(p+19) == 0x30)
			    unit_turn_on(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+j)));
		    else if(*(p+19) == 0x31)
			    unit_turn_off(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+j)));
			}
			
			for(j=0;j<segment2_num;j++)
			{
				if(*(p+19) == 0x30)
			    unit_turn_on(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+segment1_num+j)));
		    else if(*(p+19) == 0x31)
			    unit_turn_off(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+segment1_num+j)));
			}
		}
	}
	else if((*(p+15) == 0x42) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//单灯开关灯
	{
	  unit_addr_bcd =  chr2hex(*(p+20))<<4;
		unit_addr_bcd = (chr2hex(*(p+21))+unit_addr_bcd)<<4;
		unit_addr_bcd = (chr2hex(*(p+22))+unit_addr_bcd)<<4;
		unit_addr_bcd =  chr2hex(*(p+23))+unit_addr_bcd;
		
		if(*(p+19) == 0x30)
			unit_turn_on(RANDOM, unit_addr_bcd);
	  else if(*(p+19) == 0x31)
		  unit_turn_off(RANDOM, unit_addr_bcd);
	}
	else 
	{
		return;
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 5);
	
	Protocol_Response(0x85, buf_temp, 5);
}

void HandleTunnelStrategy(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[20]={0};
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!Protocol_Check(p, &data_size))
    return;
	
	MemStorage_Convert(p+15, data_size, WriteBuff);
	TunnelStrategyRun(WriteBuff);
	NorFlashWrite(NORFLASH_GATEWAY_STRATEGY_BASE, WriteBuff, data_size);
	
	Protocol_Response(0xA2, NULL, 0);	
}

void AllParaInit(void)
{
	ReadRTC_Time(RTC_Format_BIN, &time);
	rise_set();
}

