#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "gsm.h"
#include "rtc.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"
#include "electric.h"
#include "ballast_comm.h"
#include "gateway_protocol.h"
#include "ballast_protocol.h"
#include "table_process.h"
#include "norflash.h"
#include "common.h"
#include "lat_longitude.h"
#include "km_ctrl.h"

extern SemaphoreHandle_t RTC_SystemRunningSemaphore;
extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];
extern LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM];
extern u8 LampDataTempBuff[MAX_LAMP_NUM][LAMP_DATA_TEMP_SIZE];
extern const char Sofeware_Version[];
extern TimeTypeDef time;
extern LampBuffType  LampAddr;
extern xQueueHandle  GPRSSendAddrQueue;
extern xQueueHandle  LampQueryAddrQueue;
extern xQueueHandle  GSM_AT_queue;
extern xQueueHandle KMCtrlQueue;

extern TaskHandle_t xBallastComm1Task;

AlarmParmTypeDef AlarmParm;

void RTC_TimeToChar(u16 *buf)
{
	TimeTypeDef time;
	ReadRTC_Time(RTC_Format_BCD, &time);

	*buf++ = hex2chr((time.year>>4) & 0x0000000F);
	*buf++ = hex2chr(time.year & 0x0000000F);
	*buf++ = hex2chr((time.mon>>4) & 0x0000000F);
	*buf++ = hex2chr(time.mon & 0x0000000F);
	*buf++ = hex2chr((time.day>>4) & 0x0000000F);
	*buf++ = hex2chr(time.day & 0x0000000F);
	*buf++ = hex2chr((time.hour>>4) & 0x0000000F);
	*buf++ = hex2chr(time.hour & 0x0000000F);
	*buf++ = hex2chr((time.min>>4) & 0x0000000F);
	*buf++ = hex2chr(time.min & 0x0000000F);
	*buf++ = hex2chr((time.sec>>4) & 0x0000000F);
	*buf   = hex2chr(time.sec & 0x0000000F);
}

ErrorStatus unit_addr_check(u16 addr, u16 *addr_hex)
{
	u16 i,j,k,l;

	i = ((addr>>12) & 0x000f);
	j = ((addr>>8) & 0x000f);
	k = ((addr>>4) & 0x000f);
	l = ((addr) & 0x000f);
	
	if(i >9) return ERROR;
	else if(j >9) return ERROR;
	else if(k >9) return ERROR;
	else if(l >9) return ERROR;
	else
	{
		*addr_hex = i*1000+j*100+k*10+l;
		
		if(*addr_hex < MAX_LAMP_NUM)
	    return SUCCESS;  
		else 
			return ERROR;
	}
}

ErrorStatus GatewayAddrCheck(u8 *addr_buf)
{
	u16 ManagemAddr[10]={0};
	u8  BroadcastAddr[]={0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, (MANAGER_ADDR_LENGTH + 1) / 2);
	
	if(strncmp((char*)addr_buf, (char*)ManagemAddr , 10) == 0)
		return SUCCESS;
  else if(strncmp((char*)addr_buf, (char*)BroadcastAddr , 10) == 0)
		return SUCCESS;
	else
		return ERROR;
}

ErrorStatus GPRS_Protocol_Check(u8 *buf, u8 *BufData_Size)
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

void GPRS_Protocol_Response(u8 Function, u8 *databuff, u8 DataLength)
{
  u8 i,checksum;
	u8 buf[250]={0};
	u16 ManagemAddr[10]={0};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, (MANAGER_ADDR_LENGTH + 1) / 2);

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

void AlarmParmUpdate(void)
{
	u16 WriteBuff[60];
	u8 temp[10]={0};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_PARA3_BASE, WriteBuff, 60);//读出告警参数
	
	if(NorflashDataCheck(WriteBuff, 60) == EMPTY)
	{
		AlarmParm.L1_VolHigh = 0x3000;
		AlarmParm.L2_VolHigh = 0x3000;
		AlarmParm.L3_VolHigh = 0x3000;
		AlarmParm.L1_VolLow = 0x1000;
		AlarmParm.L2_VolLow = 0x1000;
		AlarmParm.L3_VolLow = 0x1000;
		AlarmParm.L1_CurLow = 0x2000;
		AlarmParm.L2_CurLow = 0x2000;
		AlarmParm.L3_CurLow = 0x2000;
		AlarmParm.Zero_CurLow = 0x2000;
		AlarmParm.L1_CurHigh = 0x5000;
		AlarmParm.L2_CurHigh = 0x5000;
		AlarmParm.L3_CurHigh = 0x5000;
		AlarmParm.Zero_CurHigh = 0x5000;
		AlarmParm.ConnectFail_Num = 0x03;	
	}
	else
	{
		ConvertToByte(WriteBuff+1, 4, temp);
		AlarmParm.L1_VolHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+5, 4, temp);
		AlarmParm.L2_VolHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+9, 4, temp);
		AlarmParm.L3_VolHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+13, 4, temp);
		AlarmParm.L1_VolLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+17, 4, temp);
		AlarmParm.L2_VolLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+21, 4, temp);
		AlarmParm.L3_VolLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+25, 4, temp);
		AlarmParm.L1_CurLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+29, 4, temp);
		AlarmParm.L2_CurLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+33, 4, temp);
		AlarmParm.L3_CurLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+37, 4, temp);
		AlarmParm.Zero_CurLow = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+41, 4, temp);
		AlarmParm.L1_CurHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+45, 4, temp);
		AlarmParm.L2_CurHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+49, 4, temp);
		AlarmParm.L3_CurHigh = atoi((const char *)temp);
		
		ConvertToByte(WriteBuff+53, 4, temp);
		AlarmParm.Zero_CurHigh = atoi((const char *)temp);
		
		AlarmParm.ConnectFail_Num = chr2hex((u8)WriteBuff[57]);
  }
}

void HandleGatewayParam(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[200]={0};
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
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
		AlarmParmUpdate();
	}
	else
	{
	  GPRS_Protocol_Response(GATEPARAM|0xC0, p+15, 1);
		return;
	}
	
	GPRS_Protocol_Response(GATEPARAM|0x80, p+15, 1);	
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
	
	if(!GPRS_Protocol_Check(p, &data_size))
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
		GPRS_Protocol_Response(LAMPPARAM|0xC0, buf_temp, 5);
		return;
	}
	
	taskENTER_CRITICAL();//进入临界区（系统可屏蔽中断）
	
	LampAddr.index = 0;
	LampAddr.num = 0;
  memset(LampAttrSortTable, 0, sizeof(LampAttrSortTable));
	LampSortFunction();
	LampRunCtrlTableClear();
	TunnelStrategyRun();
	
	taskEXIT_CRITICAL();//退出临界区
	
	for(i=0;i<lamp_param_num;i++)
	{
		for(j=0;j<4;j++)
		{
			buf_temp[n] = *(p+16+17*i+j);
			n++;
		}
	}
	buf_temp[4*lamp_param_num] = *(p+15);
	GPRS_Protocol_Response(LAMPPARAM|0x80, buf_temp, 4*lamp_param_num+1);
}

void HandleLampStrategy(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[200]={0};
	u16 i,unit_addr_hex;
	u8 buf_temp[10];
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
		return;
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))
	{
		RTC_TimeToChar(WriteBuff+STRATEGY_UNIT_BASE/2);//策略同步标识
		MemStorage_Convert(p+23, data_size-8, WriteBuff+STRATEGY_UNIT_BASE/2+12);
		
		for(i=0;i<LampAddr.num;i++)
		{
			unit_addr_check(LampAttrSortTable[i].addr, &unit_addr_hex);
			NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, PARAM_SIZE/2);//读出灯参数
			NorFlashWrite(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, WriteBuff, 200);//写入灯参数和策略
		}
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 10);
	
	GPRS_Protocol_Response(LAMPSTRATEGY|0x80, buf_temp, 10);
}

void branch_state_update(u8 branch, u8 state)//强制开接触器 更新回路单灯状态
{
	u16 unit_addr_hex;
	u16 index1 = 0xFFFF;
	u16 index2 = 0xFFFF;
	u8 enter=0,dimming_value=0;
	u16 i;
	u8 repeat_count;
	
	while(1)//地址查找
	{
		if((LampAttrSortTable[i].branch == branch)&& (enter == 0))
		{
			index1 = i;
			enter = 1;
		}
		else if((enter == 1) && (LampAttrSortTable[i].branch != branch))
		{
			index2 = i-1;
			break;
		}
		
		i++;
		
		if(i >= MAX_LAMP_NUM)
		{
			index2 = i-1;
			break;
		}
	}
	
	if(index1 == 0xFFFF)//没有相关回路或支路
	  return;
	
  for(i=index1;i<=index2;i++)//状态控制
	{
		unit_addr_hex = Bcd2ToByte(LampAttrSortTable[i].addr>>8)*100 + Bcd2ToByte(LampAttrSortTable[i].addr & 0X00FF);
		
		if(state == HARDWARE_CLOSE)
			repeat_count = 1;
		else
			repeat_count = MAX_QUERY_NUM;
		
		LampRunCtrlTableUpdate(unit_addr_hex, &repeat_count, NULL, &state, &dimming_value);
		
		xQueueSend(LampQueryAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//轮询相应的zigbee地址
	}
}

void unit_ctrl(u8 branch, u8 segment, u16 addr, u8 cmd, u8 data)//软开关单灯 主辅道 调光
{
	u16 unit_addr_hex;
	u16 index1 = 0xFFFF;
	u16 index2 = 0xFFFF;
	u8 enter=0;
	u16 i;
	
//	vTaskSuspendAll();//挂起任务调度器
	
	if((branch == ALL_BRANCH) && (segment == ALL_SEGMENT))
	{
		for(i=0;i<LampAddr.num;i++)
		{
      unit_addr_hex = Bcd2ToByte(LampAttrSortTable[i].addr>>8)*100 + Bcd2ToByte(LampAttrSortTable[i].addr & 0X00FF);
			
			if(cmd == LAMPDIMMING)
			{
				LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
				LampRunCtrlTable[unit_addr_hex].dimming_value = data;
			}
			else if(cmd == LAMPONOFF)
			{
				if(data == 0x30)
				{
					if(LampRunCtrlTable[unit_addr_hex].dimming_value != 0)
						LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
					else
						LampRunCtrlTable[unit_addr_hex].run_state = MAINRUN_FULL;
				}
				else if(data == 0x31)
				{
					LampRunCtrlTable[unit_addr_hex].run_state = SOFTWARE_CLOSE;
				}
		  }
			else if(cmd == READLAMPDATA)
			{
				xQueueSend(GPRSSendAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//直接发送单灯数据
			}
			else
				return;
			
			if(cmd != READLAMPDATA)
			{
				LampRunCtrlTable[unit_addr_hex].run_scheme = MANUAL_RUN;//手动控制标志
			  xQueueSend(LampQueryAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//轮询单灯数据
			}
		}
	}
	else if((branch == RANDOM) && (segment == RANDOM))
	{
	  if(!unit_addr_check(addr, &unit_addr_hex))
			return;
		
		if(cmd == LAMPDIMMING)
		{
			LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
			LampRunCtrlTable[unit_addr_hex].dimming_value = data;
		}
		else if(cmd == LAMPONOFF)
		{
			if(data == 0x30)
			{
				if(LampRunCtrlTable[unit_addr_hex].dimming_value != 0)
					LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
				else
					LampRunCtrlTable[unit_addr_hex].run_state = MAINRUN_FULL;
			}
			else if(data == 0x31)
				LampRunCtrlTable[unit_addr_hex].run_state = SOFTWARE_CLOSE;
	  }
		else if(cmd == READLAMPDATA)
		{
		}
		else 
			return;
		
		xQueueSend(LampQueryAddrQueue, &addr, configTICK_RATE_HZ*5);//轮询单灯数据
		if(cmd != READLAMPDATA)
		{
			LampRunCtrlTable[unit_addr_hex].run_scheme = MANUAL_RUN;//手动控制标志
		}
	}
	else
	{
		i = 0;
		
		while(1)//地址查找
		{
			if((LampAttrSortTable[i].branch == branch) && (LampAttrSortTable[i].segment == segment) && (enter == 0))
			{
				index1 = i;
				enter = 1;
			}
			else if((enter == 1) && ((LampAttrSortTable[i].branch != branch) || (LampAttrSortTable[i].segment != segment)))
			{
				index2 = i-1;
				break;
			}
			
			i++;
			
			if(i >= MAX_LAMP_NUM)
			{
				index2 = i-1;
				break;
			}
		}
		
		if(index1 == 0xFFFF)//没有相关回路或支路
			return;
		
		for(i=index1;i<=index2;i++)//状态控制
		{
			unit_addr_hex = Bcd2ToByte(LampAttrSortTable[i].addr>>8)*100 + Bcd2ToByte(LampAttrSortTable[i].addr & 0X00FF);
			
			if(cmd == LAMPDIMMING)
		  {
				LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
				LampRunCtrlTable[unit_addr_hex].dimming_value = data;
		  }
			else if(cmd == LAMPONOFF)
			{
				if(data == 0x30)
				{
					if(LampRunCtrlTable[unit_addr_hex].dimming_value != 0)
						LampRunCtrlTable[unit_addr_hex].run_state = DIMMING_RUN;
					else
						LampRunCtrlTable[unit_addr_hex].run_state = MAINRUN_FULL;
				}
				else if(data == 0x31)
					LampRunCtrlTable[unit_addr_hex].run_state = SOFTWARE_CLOSE;
		  }
			else if(cmd == READLAMPDATA)
		  {
				xQueueSend(GPRSSendAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ);//直接发送单灯数据
		  }
			else
				return;
			
			if(cmd != READLAMPDATA)
			{
				LampRunCtrlTable[unit_addr_hex].run_scheme = MANUAL_RUN;//手动控制标志
			  xQueueSend(LampQueryAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ);//轮询单灯数据
			}
		}
	}
//	vTaskDelay(1000/portTICK_RATE_MS);//延迟1s采样单灯数据
//	xTaskResumeAll ();//恢复任务调度器
}

void HandleLampDimmer(u8 *p)
{
	u8 data_size=0;
	u8 branch_num,segment1_num,segment2_num,data;
	u8 i,j;
	u8 buf_temp[30];
	u16 unit_addr_bcd;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	vTaskSuspend(xBallastComm1Task);
	vTaskDelay(1000/portTICK_RATE_MS);//延迟1000ms
	CtrlUnitSend(p+11, data_size+4);//ZIGBEE发送调光指令
	vTaskResume(xBallastComm1Task);
	
	data = chr2hex(*(p+19))<<4 | chr2hex(*(p+20));
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//网关调光
	{
	  unit_ctrl(ALL_BRANCH, ALL_SEGMENT, 0, LAMPDIMMING, data);
	}
	else if(*(p+15) == 0x39)//回路属性调光
	{
		branch_num = chr2hex(*(p+16));
		segment1_num = chr2hex(*(p+17));
		segment2_num = chr2hex(*(p+18));
		for(i=0;i<branch_num;i++)
		{
			for(j=0;j<segment1_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+21+i)), chr2hex(*(p+21+branch_num+j)), 0, LAMPDIMMING, data);
			}
			
			for(j=0;j<segment2_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+21+i)), chr2hex(*(p+21+branch_num+segment1_num+j))|0x10, 0, LAMPDIMMING, data);
			}
		}
	}
	else if((*(p+15) == 0x42) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//单灯调光
	{
	  unit_addr_bcd =  chr2hex(*(p+21))<<4;
		unit_addr_bcd = (chr2hex(*(p+22))+unit_addr_bcd)<<4;
		unit_addr_bcd = (chr2hex(*(p+23))+unit_addr_bcd)<<4;
		unit_addr_bcd =  chr2hex(*(p+24))+unit_addr_bcd;
		
		unit_ctrl(RANDOM, RANDOM , unit_addr_bcd, LAMPDIMMING, data);
	}
	else 
	{
		return;
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 6);
	GPRS_Protocol_Response(LAMPDIMMING|0x80, buf_temp, 6);
}

void HandleLampOnOff(u8 *p)
{
	u8 data_size=0;
	u8 branch_num,segment1_num,segment2_num,data;
	u8 i,j;
	u8 buf_temp[30];
	u16 unit_addr_bcd;
	
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	vTaskSuspend(xBallastComm1Task);
	vTaskDelay(1000/portTICK_RATE_MS);//延迟1000ms
	CtrlUnitSend(p+11, data_size+4);//ZIGBEE发送单灯开关指令
	vTaskResume(xBallastComm1Task);
	
	data = *(p+19);
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//网关开关灯
	{
	  unit_ctrl(ALL_BRANCH, ALL_SEGMENT, 0, LAMPONOFF, data);
	}
	else if(*(p+15) == 0x39)//回路属性开关灯
	{
		branch_num = chr2hex(*(p+16));
		segment1_num = chr2hex(*(p+17));
		segment2_num = chr2hex(*(p+18));
		for(i=0;i<branch_num;i++)
		{
			for(j=0;j<segment1_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+j)), 0, LAMPONOFF, data);
			}
			
			for(j=0;j<segment2_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+20+i)), chr2hex(*(p+20+branch_num+segment1_num+j))|0x10, 0, LAMPONOFF, data);
			}
		}
	}
	else if((*(p+15) == 0x42) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//单灯开关灯
	{
	  unit_addr_bcd =  chr2hex(*(p+20))<<4;
		unit_addr_bcd = (chr2hex(*(p+21))+unit_addr_bcd)<<4;
		unit_addr_bcd = (chr2hex(*(p+22))+unit_addr_bcd)<<4;
		unit_addr_bcd =  chr2hex(*(p+23))+unit_addr_bcd;
		
		unit_ctrl(RANDOM, RANDOM , unit_addr_bcd, LAMPONOFF, data);
	}
	else 
	{
		return;
	}

	strncpy((char*)buf_temp, (char*)p+15, 5);
	GPRS_Protocol_Response(LAMPONOFF|0x80, buf_temp, 5);
}

void HandleUnitRunBack(u8 *p)
{
	u8 data_size=0;
	u8 buf_temp[30];
//	u16 i;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//网关开关灯
	{
	  LampRunCtrlTableClear();
		TunnelStrategyRun();//顺序不能反
//		for(i=0;i<LampAddr.num;i++)
//		{
//			xQueueSend(LampQueryAddrQueue, &LampAttrSortTable[i].addr, configTICK_RATE_HZ*5);//轮询单灯数据
//		}
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 4);
	GPRS_Protocol_Response(UNITRUNBACK|0x80, buf_temp, 4);
}

void GPRSSendUnitDataFun(u16 *addr_bcd, u8 num, u16 pro_type)
{
	u16 unit_addr_hex;
  u8 buf[250];
	u8 i,index=0;
	
	buf[index++] = 0x41;
	buf[index++] = 0x30;
	buf[index++] = 0x30;
	buf[index++] = 0x30;
	
	for(i=0;i<num;i++)
	{
		unit_addr_hex = Bcd2ToByte(*(addr_bcd+i)>>8)*100 + Bcd2ToByte(*(addr_bcd+i) & 0x00FF);

		taskENTER_CRITICAL();//进入临界区（系统可屏蔽中断）
		strncpy((char*)&buf[index], (char*)LampDataTempBuff[unit_addr_hex], 20);
		taskEXIT_CRITICAL();//退出临界区
		index += 20;
		
	  if(pro_type == 0x86)
		{
			taskENTER_CRITICAL();//进入临界区（系统可屏蔽中断）
			strncpy((char*)&buf[index], (char*)LampDataTempBuff[unit_addr_hex]+20, 24);
			taskEXIT_CRITICAL();//退出临界区
			index += 24;
		}
		else if(pro_type == 0xA6)
		{
			continue;
		}
		else
		{
			return;
		}
	}
	
	GPRS_Protocol_Response(pro_type, buf, index);
}

void HandleReadBSNData(u8 *p)
{
	u8 data_size=0;
	u8 branch_num,segment1_num,segment2_num;
	u8 i,j;
	u8 buf_temp[30];
	u16 unit_addr_bcd;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	if((*(p+15) == 0x41) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//读网关下所有灯数据
	{
		unit_ctrl(ALL_BRANCH, ALL_SEGMENT, 0, READLAMPDATA, 0);
	}
	else if(*(p+15) == 0x39)//回路属性镇流器数据
	{
		branch_num = chr2hex(*(p+16));
		segment1_num = chr2hex(*(p+17));
		segment2_num = chr2hex(*(p+18));
		for(i=0;i<branch_num;i++)
		{
			for(j=0;j<segment1_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+19+i)), chr2hex(*(p+19+branch_num+j)), 0, READLAMPDATA, 0);
			}
			
			for(j=0;j<segment2_num;j++)
			{
			  unit_ctrl(chr2hex(*(p+19+i)), chr2hex(*(p+19+branch_num+segment1_num+j))|0x10, 0, READLAMPDATA, 0);
			}
		}
	}
	else if((*(p+15) == 0x42) && (*(p+16) == 0x30) && (*(p+17) == 0x30) && (*(p+18) == 0x30))//单灯镇流器数据
	{
	  unit_addr_bcd =  chr2hex(*(p+19))<<4;
		unit_addr_bcd = (chr2hex(*(p+20))+unit_addr_bcd)<<4;
		unit_addr_bcd = (chr2hex(*(p+21))+unit_addr_bcd)<<4;
		unit_addr_bcd =  chr2hex(*(p+22))+unit_addr_bcd;
		
		unit_ctrl(RANDOM, RANDOM , unit_addr_bcd, READLAMPDATA, 0);
	}
	else 
	{
		return;
	}
	
	strncpy((char*)buf_temp, (char*)p+15, 4);
	
	GPRS_Protocol_Response(READLAMPDATA|0x80, buf_temp, 4);
}

void HandleBranchOnOff(u8 *p)
{
	u8 data_size=0;
	u8 *buf;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	if(*(p+15) == 0x30)
	{
		buf = p+15;
		xQueueSend(KMCtrlQueue, buf, configTICK_RATE_HZ);
	}
}

void HandleGWDataQuery(u8 *p)
{
	u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	EleDMA_TxBuff((char*)p, data_size+18);
}

void HandleAdjustTime(u8 *p)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
  RTC_TimeTypeDef RTC_TimeTypeInitStructure;
  u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	RTC_DateTypeInitStructure.RTC_Year = Bcd2ToByte(chr2hex(*(p+15))<<4 | chr2hex(*(p+16)));
	RTC_DateTypeInitStructure.RTC_Month = Bcd2ToByte(chr2hex(*(p+17))<<4 | chr2hex(*(p+18)));
	RTC_DateTypeInitStructure.RTC_Date = Bcd2ToByte(chr2hex(*(p+19))<<4 | chr2hex(*(p+20)));
  RTC_TimeTypeInitStructure.RTC_Hours = Bcd2ToByte(chr2hex(*(p+21))<<4 | chr2hex(*(p+22)));
	RTC_TimeTypeInitStructure.RTC_Minutes = Bcd2ToByte(chr2hex(*(p+23))<<4 | chr2hex(*(p+24)));
	RTC_TimeTypeInitStructure.RTC_Seconds = Bcd2ToByte(chr2hex(*(p+25))<<4 | chr2hex(*(p+26)));
	
	RTC_DateTypeInitStructure.RTC_WeekDay = CaculateWeekDay(((u16)RTC_DateTypeInitStructure.RTC_Year + RTC_ReadBackupRegister(RTC_BKP_DR1)*100), 
	                                        RTC_DateTypeInitStructure.RTC_Month, RTC_DateTypeInitStructure.RTC_Date);
	
	if(xSemaphoreTake(RTC_SystemRunningSemaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
	  RTC_SetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
		RTC_SetDate(RTC_Format_BIN,&RTC_DateTypeInitStructure);
		
		vTaskDelay(1);
		xSemaphoreGive(RTC_SystemRunningSemaphore);
	}
}

void HandleGWVersQuery(u8 *p)
{
	u8 data_size=0;
	char buf[20];
	char buf_temp[5];
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	sscanf(Sofeware_Version, "%*[^:]:%s", buf);
	sscanf(buf,"V%c.%c%c", &buf_temp[0], &buf_temp[1], &buf_temp[2]);
	
	GPRS_Protocol_Response(VERSIONQUERY|0x80, (u8*)buf_temp, 3);
}

void HandleElecVersQuery(u8 *p)
{
	u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	EleDMA_TxBuff((char*)p, data_size+18);
}

void HandleGWAddrQuery(u8 *p)
{
	u8 data_size=0;
	u8 buf_temp[20];
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	strncpy((char*)buf_temp, (char*)p+15, 16);
	GPRS_Protocol_Response(GWADDRQUERY|0x80, buf_temp, 16);
}

void HandleSetIPPort(u8 *p)
{
	WG_ServerParameterType   DebugWG_ServerPara;
	char message[30];
	char buff[30] = {0};
	char endchar = 0;
	short buf[4] = {0};
	int port = 0;
	u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	strncpy((char*)message, (char*)p+15, data_size);
	if(6 == sscanf(message,"%hd.%hd.%hd.%hd,%d%c",&buf[0],&buf[1],&buf[2],&buf[3],&port,&endchar))
	{
		if((0<=buf[0] && buf[0]<=255) && (0<=buf[1] && buf[1]<=255) && 
			 (0<=buf[2] && buf[2]<=255) && (0<=buf[3] && buf[3]<=255) &&
			 (0<=port   && port<=65535) && (endchar == ';'))
		{
			sscanf(message, "%[^,]", DebugWG_ServerPara.serverIP);//存储IP地址及端口按照字符类型存储
			sscanf(message, "%*[^,],%[^;]", DebugWG_ServerPara.serverPORT); 
			strncpy(buff, (char*)DebugWG_ServerPara.serverIP, sizeof(DebugWG_ServerPara.serverIP));
			strncpy(buff+16, (char*)DebugWG_ServerPara.serverPORT, sizeof(DebugWG_ServerPara.serverPORT));
			
			NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16*)buff, sizeof(buff)/2);
		}
		else
			return;
	}
	
	GPRS_Protocol_Response(SETSERVERIPPORT|0x80, NULL, 0);
	
	vTaskDelay(configTICK_RATE_HZ);
	NVIC_SystemReset();
}

void HandleGWUpgrade(u8 *P)
{
	
}

void CSQ_Reply(char *p)
{
	int signel_value,error_rate;
	u8 buf_temp[5];
	
	if(SwitchToData() == ERROR)
	{
		if(GsmStartConnect() == ERROR)
			NVIC_SystemReset();
	}
	
  sscanf(p,"%*[^:]:%x,%x", &signel_value, &error_rate);
	
	buf_temp[0] = hex2chr((signel_value>>4) & 0x0F);
	buf_temp[1] = hex2chr(signel_value & 0x0F);
	buf_temp[2] = hex2chr((error_rate>>4) & 0x0F);
	buf_temp[3] = hex2chr(error_rate & 0x0F);
	
	GPRS_Protocol_Response(GPRSQUALITY|0x80, buf_temp, 4);//信号强度表示方法
}

void HandleSignalQuality(u8 *p)
{
	u8 data_size;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	if(SwitchToCommand() == SUCCESS)
	{
		if(!SendMsgToSim("AT+CSQ\r\n", "+CSQ:", configTICK_RATE_HZ*2))
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}
  }
	else
	{
		if(GsmStartConnect() == ERROR)
		  NVIC_SystemReset();
	}
}

void HandleTunnelStrategy(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[20]={0};
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	MemStorage_Convert(p+15, data_size, WriteBuff);
	NorFlashWrite(NORFLASH_GATEWAY_STRATEGY_BASE, WriteBuff, data_size);
	TunnelStrategyRun();
	
	GPRS_Protocol_Response(TUNNELSTRATEGY|0x80, NULL, 0);	
}


void HandleRestart(u8 *p)
{
	u8 data_size=0;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	vTaskDelay(configTICK_RATE_HZ);
	
	if(*(p+15) == 0x31)
	{
		NVIC_SystemReset();
	}
	else if(*(p+15) == 0x32)
	{
		if(GsmStartConnect() == SUCCESS)
		{
			GPRS_Protocol_Response(RESTART|0x80, p+15, 1);
		}
		else
		{
			NVIC_SystemReset();
		}
	}
	else if(*(p+15) == 0x33)
	{
		EleDMA_TxBuff((char*)p, data_size+18);
		GPRS_Protocol_Response(RESTART|0x80, p+15, 1);
	}
}

void HandleSend(u8 *p)
{
	u8 data_size=0;
	u8 function;
	
	if(!GPRS_Protocol_Check(p, &data_size))
		return;
			
	function = (chr2hex(*(p+11))<<4 | chr2hex(*(p+12)));
	GPRS_Protocol_Response(function, p+15, data_size);
}

void AllParaInit(void)
{
	rise_set();
	AlarmParmUpdate();
}

