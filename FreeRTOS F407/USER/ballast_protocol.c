#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "ballast_comm.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "gsm.h"
#include "rtc.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "uart_debug.h"
#include "norflash.h"
#include "common.h"
#include "ballast_protocol.h"
#include "gateway_protocol.h"
#include "table_process.h"
#include "lat_longitude.h"
#include "electric.h"

extern u8 LampDataTempBuff[MAX_LAMP_NUM][LAMP_DATA_TEMP_SIZE];
extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];
extern LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM];
extern xQueueHandle  GPRSSendAddrQueue;
extern xQueueHandle  GSM_GPRS_queue;
extern LampBuffType LampAddr;
extern SUN_RISE_SET  sun_rise_set;
extern u8 UnitWaitFlag;
extern u8 UnitQueryState;
extern AlarmParmTypeDef AlarmParm;
u16 QueryUnitAddrBCD;
static u16 CommErrorBuff_1[20]={0},CommErrorAddr[20]={0};
extern WG_AlarmFlagDef WG_AlarmFlag;
static u8 UnitErrorNum = 0;
static u32 UnitQueryCount=0,LastAlarmTime=0;

ErrorStatus Lamp_Protocol_Check(u8 *buf, u8 *BufData_Size)
{
	u8 i,p_size=0;
	ErrorStatus state = ERROR;
	
	*BufData_Size = (chr2hex(*(buf+7))<<4 | chr2hex(*(buf+8)));
	
  for(i=0;i<255;i++)
	{
		p_size++;
		if(*(buf++) == 0x03)
			break;
	}
	 
	if((*BufData_Size + 12) == p_size)
	{
	  state = SUCCESS;
	}
	return state;
}

void unit_comm_fail_check(void)
{
	u8 WG_query[]= {0x02,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x30,0x38,0x30,0x31,0x30,0x33,0x42,0x03};
	u16 unit_addr_hex,i;
	u16 index=0xFFFF,num=0,start_index=0;
	u8 n,j,error_state=0,serv_state = ERROR,unit_state;
	u8 UnitBuff[LAMP_DATA_TEMP_SIZE];
	
	if(CommErrorAddr[0] == 0)//µÆ²»ÁÁµØÖ·Ð´Èë
	{
		if(CommErrorBuff_1[0] == 0)//µÚÒ»È¦ÂÖÑ¯
		{
			for(i=0;i<LampAddr.num;i++)
			{
				unit_addr_hex = Bcd2ToByte(LampAttrSortTable[i].addr>>8)*100 + Bcd2ToByte(LampAttrSortTable[i].addr & 0x00FF);
				
				taskENTER_CRITICAL();//½øÈëÁÙ½çÇø
				strncpy((char*)UnitBuff, (char*)LampDataTempBuff[unit_addr_hex], LAMP_DATA_TEMP_SIZE);//¶Á³öÉÏÒ»´Îµ¥µÆÊý¾Ý
				taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
	
	      unit_state = (chr2hex(UnitBuff[6])<<4 | chr2hex(UnitBuff[7]));
				
				if((unit_state & 0x10) == 0x10)
				{
					if(index == 0xFFFF)
					{
						start_index = i;
						index = i;
						num++;
					}
					else if((index != 0xFFFF) && (i == (index+1)))
					{
						index = i;
						num++;
						
					  if(num >= AlarmParm.ConnectFail_Num)
					  {
					    for(j=0;j<AlarmParm.ConnectFail_Num;j++)
						  {
							  unit_addr_hex = Bcd2ToByte(LampAttrSortTable[start_index+j].addr>>8)*100 + Bcd2ToByte(LampAttrSortTable[start_index+j].addr & 0x00FF);

								CommErrorBuff_1[j] = LampAttrSortTable[start_index+j].addr;
								LampRunCtrlTable[unit_addr_hex].query_num = 2*MAX_QUERY_NUM;
						  }
							return;
					  }
					}
					else
					{
						index = 0xFFFF;
						start_index = 0;
						num = 0;
					}
				}
				else if(index != 0xFFFF)
				{
					index = 0xFFFF;
					start_index = 0;
					num = 0;
				}
			}
		}
		else if(CommErrorAddr[0] == 0)//µÚ¶þÈ¦ÂÖÑ¯
		{
			error_state = 1;
			for(n=0;n<AlarmParm.ConnectFail_Num;n++)
			{
				unit_addr_hex = Bcd2ToByte(CommErrorBuff_1[n]>>8)*100 + Bcd2ToByte(CommErrorBuff_1[n] & 0x00FF);
				
				taskENTER_CRITICAL();//½øÈëÁÙ½çÇø
				strncpy((char*)UnitBuff, (char*)LampDataTempBuff[unit_addr_hex], LAMP_DATA_TEMP_SIZE);//¶Á³öÉÏÒ»´Îµ¥µÆÊý¾Ý
				taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
				
				unit_state = (chr2hex(UnitBuff[6])<<4 | chr2hex(UnitBuff[7]));
				
				if((unit_addr_hex == 0) || ((unit_state&0x10) == 0x00))
				{
					error_state = 0;
				}
			}
			if(error_state == 1)
			{
				for(n=0;n<AlarmParm.ConnectFail_Num;n++)
				{
					CommErrorAddr[n] = CommErrorBuff_1[n];
				}
				
        WG_AlarmFlag.cont_off = 1;
				xQueueSend(GSM_GPRS_queue, &WG_query, configTICK_RATE_HZ*5);//Íø¹ØÊý¾Ý²éÑ¯
			}
			memset((char*)CommErrorBuff_1, 0, sizeof(CommErrorBuff_1));//Çå¿ÕµÚÒ»´ÎÂÖÑ¯²»ÁÁbuf
		}
  }
	else//²é¿´ÊÇ·ñÎ¬»¤
	{
		serv_state = SUCCESS;
		
		for(n=0;n<AlarmParm.ConnectFail_Num;n++)
		{
			unit_addr_hex = Bcd2ToByte(CommErrorAddr[n]>>8)*100 + Bcd2ToByte(CommErrorAddr[n] & 0x00FF);
			
			taskENTER_CRITICAL();//½øÈëÁÙ½çÇø
			strncpy((char*)UnitBuff, (char*)LampDataTempBuff[unit_addr_hex], LAMP_DATA_TEMP_SIZE);//¶Á³öÉÏÒ»´Îµ¥µÆÊý¾Ý
			taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
				
			unit_state = (chr2hex(UnitBuff[6])<<4 | chr2hex(UnitBuff[7]));
			
			if((unit_addr_hex == 0) || ((unit_state&0x10) == 0x10))
			{
				serv_state = ERROR;
			}
		}
		if(serv_state == SUCCESS)
		{
			WG_AlarmFlag.cont_off = 0;
			xQueueSend(GSM_GPRS_queue, &WG_query, configTICK_RATE_HZ*5);//Íø¹ØÊý¾Ý²éÑ¯
			memset((char*)CommErrorAddr, 0, sizeof(CommErrorAddr));//Çå¿ÕÁ¬ÐøµÆ²»ÁÁÊý¾Ý
		}
	}
}

void QueryNextAddr(void)
{
	u8 WG_query[]= {0x02,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x30,0x38,0x30,0x31,0x30,0x33,0x42,0x03};
		
	if(LampAddr.index < (LampAddr.num-1))
	{
		LampAddr.index++;
	}
	else
	{
		LampAddr.index = 0;
		UnitQueryCount++;
	}
	
	if(LampAttrSortTable[LampAddr.index].addr == 0)//ÎÞµÆ²ÎÊý
		return;
	
	if((UnitErrorNum < AlarmParm.ConnectFail_Num) && (LampAddr.num >= AlarmParm.ConnectFail_Num))
	{
		UnitErrorNum++;
	}
		
	if(UnitErrorNum >= AlarmParm.ConnectFail_Num)
	{
		LastAlarmTime = UnitQueryCount;
		if(WG_AlarmFlag.cont_off == 0)
		{
			WG_AlarmFlag.cont_off = 1;
			xQueueSend(GSM_GPRS_queue, &WG_query, configTICK_RATE_HZ*5);//Íø¹ØÊý¾Ý²éÑ¯
		}
	}
	else if((UnitQueryCount - LastAlarmTime) >= 2)
	{
		if(WG_AlarmFlag.cont_off == 1)
		{
			WG_AlarmFlag.cont_off = 0;
			xQueueSend(GSM_GPRS_queue, &WG_query, configTICK_RATE_HZ*5);//Íø¹ØÊý¾Ý²éÑ¯
		}
	}
	
	ReadUnitData(LampAttrSortTable[LampAddr.index].addr);
}

void LampTimeReload(void)
{
	TimeTypeDef time;
	u8 buf[50];
	u8 checksum;
	
	vTaskDelay(10/portTICK_RATE_MS);//ÑÓ³Ù10ms 
	
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	
	buf[2] = 0x02;
	
	buf[3] = 0x46;
	buf[4] = 0x46;
	buf[5] = 0x46;
	buf[6] = 0x46;
	
	buf[7] = hex2chr((UNITTIMEUPATE>>4) & 0x0F);
	buf[8] = hex2chr(UNITTIMEUPATE & 0x0F);
	
	buf[9] = 0x31;
	buf[10] = 0x32;
	
	ReadRTC_Time(RTC_Format_BCD, &time); //Ê±¼äÐ£Ñé
	
	buf[11] = hex2chr((time.mon>>4) & 0x0F);
	buf[12] = hex2chr(time.mon & 0x0F);
	buf[13] = hex2chr((time.day>>4) & 0x0F);
	buf[14] = hex2chr(time.day & 0x0F);
	buf[15] = hex2chr((time.week>>4) & 0x0F);
	buf[16] = hex2chr(time.week & 0x0F);
	
	buf[17] = hex2chr((time.hour>>4) & 0x0F);
	buf[18] = hex2chr(time.hour & 0x0F);
	buf[19] = hex2chr((time.min>>4) & 0x0F);
	buf[20] = hex2chr(time.min & 0x0F);

	buf[21] = hex2chr((ByteToBcd2(sun_rise_set.rise_hour)>>4) & 0x0F);
	buf[22] = hex2chr(ByteToBcd2(sun_rise_set.rise_hour) & 0x0F);
	buf[23] = hex2chr((ByteToBcd2(sun_rise_set.rise_min)>>4) & 0x0F);
	buf[24] = hex2chr(ByteToBcd2(sun_rise_set.rise_min) & 0x0F);
	
	buf[25] = hex2chr((ByteToBcd2(sun_rise_set.set_hour)>>4) & 0x0F);
	buf[26] = hex2chr(ByteToBcd2(sun_rise_set.set_hour) & 0x0F);
	buf[27] = hex2chr((ByteToBcd2(sun_rise_set.set_min)>>4) & 0x0F);
	buf[28] = hex2chr(ByteToBcd2(sun_rise_set.set_min) & 0x0F);
	
  checksum = BCC_CheckSum(buf+2, 27);
	buf[29] = hex2chr((checksum>>4)&0x0f);
  buf[30] = hex2chr( checksum&0x0f    );
	buf[31] = 0x03;
	
	UnitComm1DMA_TxBuff(buf, 32);
}

void LampParamReload(u16 unit_addr_bcd)
{
	u16 ReadBuff[PARAM_SIZE/2];
	u16 unit_addr_hex;
	u8 buf[50];
	u8 checksum;
	
	vTaskDelay(10/portTICK_RATE_MS);//ÑÓ³Ù10ms 
	
	unit_addr_hex = Bcd2ToByte(unit_addr_bcd>>8)*100 + Bcd2ToByte(unit_addr_bcd & 0x00FF);
	NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE, ReadBuff, PARAM_SIZE/2);//¶Á³öµÆ²ÎÊý
	
	buf[0] = (u8)(unit_addr_bcd>>8);
	buf[1] = (u8)(unit_addr_bcd & 0x00FF);
	
	buf[2] = 0x02;
	
  ConvertToByte(ReadBuff, 4 , buf+3);//ZigBeeµØÖ·
	
	buf[7] = hex2chr((UNITPARAM>>4) & 0x0F);
	buf[8] = hex2chr(UNITPARAM & 0x0F);
	
	buf[9] = 0x32;
	buf[10] = 0x35;
	
  ConvertToByte(ReadBuff+4, 25 , buf+11);//²ÎÊýÍ¬²½±êÊ¶
	
	checksum = BCC_CheckSum(buf+2, 34);
	buf[36] = hex2chr((checksum>>4)&0x0f);
  buf[37] = hex2chr( checksum&0x0f    );
	buf[38] = 0x03;
	
	UnitWaitFlag = WAIT_PARAM_REPLY;
	UnitComm1DMA_TxBuff(buf, 39);
}

void LampStrategyReload(u16 unit_addr_bcd)
{
	u16 ReadBuff[DAY_STRATEGY_SIZE/2];
	u16 unit_addr_hex;
	u8 buf[50],data_size;
	u8 checksum,i;
	
	vTaskDelay(10/portTICK_RATE_MS);//ÑÓ³Ù10ms 
	
	unit_addr_hex = Bcd2ToByte(unit_addr_bcd>>8)*100 + Bcd2ToByte(unit_addr_bcd & 0x00FF);
	NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE + STRATEGY_UNIT_BASE, ReadBuff, DAY_STRATEGY_SIZE/2);//¶Á³öµÆ²ÎÊý
	
	for(i=0;i<200;i++)
	{
		if((ReadBuff[i] == 0xFFFF) || (ReadBuff[i] == 0x0000))
		{
			data_size = i;
			break;
		}
	}
	buf[0] = (u8)(unit_addr_bcd>>8);
	buf[1] = (u8)(unit_addr_bcd & 0x00FF);
	
	buf[2] = 0x02;
	
	buf[3] = hex2chr((unit_addr_bcd >> 12) & 0x000F);
	buf[4] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	buf[5] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	buf[6] = hex2chr((unit_addr_bcd) & 0x000F);
	
	buf[7] = hex2chr((UNITSTRATEGY>>4) & 0x0F);
	buf[8] = hex2chr(UNITSTRATEGY & 0x0F);
	
	buf[9] = hex2chr((data_size>>4) & 0x0F);
	buf[10] = hex2chr(data_size & 0x0F);
	
  ConvertToByte(ReadBuff, data_size , buf+11);//²ßÂÔÊý¾Ý
	
	checksum = BCC_CheckSum(buf+2, data_size+9);
	buf[11+data_size] = hex2chr((checksum>>4)&0x0f);
  buf[11+data_size+1] = hex2chr( checksum&0x0f  );
	buf[11+data_size+2] = 0x03;
	
	UnitWaitFlag = WAIT_STRATEGY_REPLY;
	UnitComm1DMA_TxBuff(buf, 11+data_size+2+1);
}

void ReadUnitData(u16 unit_addr_bcd)
{
	vTaskDelay(10/portTICK_RATE_MS);//ÑÓ³Ù10ms 
	
	QueryUnitAddrBCD = unit_addr_bcd;
	
	u8 buf[50];
	u8 checksum;
	
	buf[0] = (u8)(unit_addr_bcd>>8);
	buf[1] = (u8)(unit_addr_bcd & 0x00FF);
	
	buf[2] = 0x02;
	
	buf[3] = hex2chr((unit_addr_bcd >> 12) & 0x000F);
	buf[4] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	buf[5] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	buf[6] = hex2chr((unit_addr_bcd) & 0x000F);
	
	buf[7] = 0x30;
	buf[8] = 0x36;
	
	buf[9] = 0x30;
	buf[10] = 0x30;
	
	checksum = BCC_CheckSum(buf+2, 9);
	buf[11] = hex2chr((checksum>>4)&0x0F);
  buf[12] = hex2chr( checksum & 0x0F  );
	buf[13] = 0x03;
	
	UnitWaitFlag = WAIT_READDATA_REPLY;
	UnitComm1DMA_TxBuff(buf, 14);
}

void CtrlUnitSend(u8 *p, u8 size)
{
	u8 buf[30],checksum;
	
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	
	buf[2] = 0x02;
	
	buf[3] = 0x46;
	buf[4] = 0x46;
	buf[5] = 0x46;
	buf[6] = 0x46;
	
  strncpy((char*)buf+7, (char*)p, size);
	
	checksum = BCC_CheckSum(buf+2, size+5);
	buf[7+size] = hex2chr((checksum>>4)&0x0F);
  buf[8+size] = hex2chr( checksum & 0x0F  );
	buf[9+size] = 0x03;
	
	UnitComm1DMA_TxBuff(buf, 10+size);
}

void UnitDimmmingCtrl(u16 unit_addr_bcd, u8 dimming_value)
{
	u8 buf[20];
	
	buf[0] = hex2chr((UNITDIMMING >> 4) & 0x000F);
	buf[1] = hex2chr(UNITDIMMING & 0x000F);
	
	buf[2] = 0x30;
	buf[3] = 0x39;
	
	buf[4] = 0x42;
	buf[5] = 0x30;
	buf[6] = 0x30;
	buf[7] = 0x30;
	
	buf[8] = hex2chr((dimming_value>>4) & 0x0F);
	buf[9] = hex2chr(dimming_value & 0x0F);
	
	buf[10] = hex2chr((unit_addr_bcd >> 12) & 0x000F);
	buf[11] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	buf[12] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	buf[13] = hex2chr(unit_addr_bcd & 0x000F);
	
	CtrlUnitSend(buf, 14);
}

void UnitOnOffCtrl(u16 unit_addr_bcd, u8 cmd)
{
	u8 buf[20];
	
	buf[0] = hex2chr((UNITONOFF >> 4) & 0x000F);
	buf[1] = hex2chr(UNITONOFF & 0x000F);
	
	buf[2] = 0x30;
	buf[3] = 0x39;
	
	buf[4] = 0x42;
	buf[5] = 0x30;
	buf[6] = 0x30;
	buf[7] = 0x30;
	
	buf[8] = cmd;
	
	buf[9] = hex2chr((unit_addr_bcd >> 12) & 0x000F);
	buf[10] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	buf[11] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	buf[12] = hex2chr(unit_addr_bcd & 0x000F);
	
	CtrlUnitSend(buf, 13);
}

void CtrlBackRun(u16 unit_addr_bcd)
{
	u8 buf[20];
	
	buf[0] = hex2chr((UNITCTRLBACK >> 4) & 0x000F);
	buf[1] = hex2chr(UNITCTRLBACK & 0x000F);
	
	buf[2] = 0x30;
	buf[3] = 0x39;
	
	buf[4] = 0x42;
	buf[5] = 0x30;
	buf[6] = 0x30;
	buf[7] = 0x30;
	
	buf[8] = hex2chr((unit_addr_bcd >> 12) & 0x000F);
	buf[9] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	buf[10] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	buf[11] = hex2chr(unit_addr_bcd & 0x000F);
	
	CtrlUnitSend(buf, 12);
}

void HandleUnitLightParamReply(u8 *p)
{
  UnitWaitFlag = WAIT_READDATA_REPLY;
}

void HandleUnitStrategyReply(u8 *p)
{
  UnitWaitFlag = WAIT_READDATA_REPLY;
}

void clear_unit_buff(u8 state, u16 unit_addr_bcd)//¹Ø±ÕÓ² »ò µ¥µÆÍ¨ÐÅÊ§Áª  Êý¾ÝÇå¿Õª
{
	u8 last_state,temp[10];
	u16 unit_addr_hex;
	u8 LastBuff[LAMP_DATA_TEMP_SIZE];
	
	unit_addr_hex = Bcd2ToByte(unit_addr_bcd>>8)*100 + Bcd2ToByte(unit_addr_bcd & 0x00FF);
	
	temp[0] = hex2chr((unit_addr_bcd >> 12) & 0x000F);//zigbeeµØÖ·
	temp[1] = hex2chr((unit_addr_bcd >> 8) & 0x000F);
	temp[2] = hex2chr((unit_addr_bcd >> 4) & 0x000F);
	temp[3] = hex2chr(unit_addr_bcd & 0x000F);
	
	temp[4] = hex2chr((state >>4) & 0x000F);//ÔËÐÐ×´Ì¬
	temp[5] = hex2chr(state & 0x000F);
	
	taskENTER_CRITICAL();//½øÈëÁÙ½çÇø

	strncpy((char*)LastBuff, (char*)LampDataTempBuff[unit_addr_hex], LAMP_DATA_TEMP_SIZE);//¶Á³öÉÏÒ»´Îµ¥µÆÊý¾Ý

	strncpy((char*)LampDataTempBuff[unit_addr_hex], (char*)temp, 4);//ZigBeeµØÖ·
	memset((char*)LampDataTempBuff[unit_addr_hex]+4, 0x30, 2);//µ÷¹âÖµ
	strncpy((char*)LampDataTempBuff[unit_addr_hex]+6, (char*)temp+4, 2);//ÔËÐÐ×´Ì¬
	memset((char*)LampDataTempBuff[unit_addr_hex]+8, 0x30, 36);//ÊµÊ±Êý¾Ý
		
	taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
	
	last_state = (chr2hex(LastBuff[6])<<4 | chr2hex(LastBuff[7]));
	if(state != last_state)
	{
		xQueueSend(GPRSSendAddrQueue, &unit_addr_bcd, configTICK_RATE_HZ*5);
	}
}

void unit_state_ctrl(u16 unit_addr_bcd, u8 now_state, u8 ctrl_state)
{
	u8 run_state;
	u16 unit_addr_hex;
	
	unit_addr_hex = Bcd2ToByte(unit_addr_bcd>>8)*100 + Bcd2ToByte(unit_addr_bcd & 0x00FF);
	
	run_state = (now_state & 0x1F);
	
	if((run_state == MAINRUN_UNFULL) && (ctrl_state == MAINRUN_FULL))
	{
		
	}
	else if(((run_state & 0x10) == 0) && ((run_state & 0x0F) != ctrl_state))//ÔËÐÐ×´Ì¬ÓëÍø¹Ø¿ØÖÆ²ßÂÔ×´Ì¬²»Ïà·û
	{
		if((ctrl_state == DIMMING_RUN) && (run_state != SMART_RUN))
		{
		  UnitDimmmingCtrl(unit_addr_bcd, LampRunCtrlTable[unit_addr_hex].dimming_value);
		}
		else if(ctrl_state == SOFTWARE_CLOSE)
		{
			UnitOnOffCtrl(unit_addr_bcd, 0x31);
		}
		else if(ctrl_state == MAINRUN_FULL)
		{
			if(run_state == DIMMING_RUN)
			  CtrlBackRun(unit_addr_bcd);
			else if(run_state == SOFTWARE_CLOSE)
				UnitOnOffCtrl(unit_addr_bcd, 0x30);
		}
	}
}


void HandleUnitReadDataReply(u8 *p)
{
	LampDataTypeDef lamp_data;
	LampDataTypeDef last_lamp_data;
	TimeTypeDef time;
	int now_power_hex,last_power_hex;
	u16 NorReadBuff[12]={0};
	u16 unit_addr_hex;
	u8 data_size,i,buf[6],LastBuff[LAMP_DATA_TEMP_SIZE];

  if(!Lamp_Protocol_Check(p, &data_size))
    return;

	UnitErrorNum = 0;//Çå³ýÁ¬ÐøµÆ²»ÁÁÊýÄ¿
	
	lamp_data.addr_bcd =  chr2hex(*(p+1))<<4;
	lamp_data.addr_bcd = (chr2hex(*(p+2))+lamp_data.addr_bcd)<<4;
	lamp_data.addr_bcd = (chr2hex(*(p+3))+lamp_data.addr_bcd)<<4;
	lamp_data.addr_bcd =  chr2hex(*(p+4))+lamp_data.addr_bcd;
	
	lamp_data.dimming = (chr2hex(*(p+9))<<4 | chr2hex(*(p+10)));
	
	lamp_data.state = (chr2hex(*(p+11))<<4 | chr2hex(*(p+12)));
	
	lamp_data.power =  chr2hex(*(p+21))<<4;
	lamp_data.power = (chr2hex(*(p+22))+lamp_data.power)<<4;
	lamp_data.power = (chr2hex(*(p+23))+lamp_data.power)<<4;
	lamp_data.power =  chr2hex(*(p+24))+lamp_data.power;
	
	unit_addr_hex = Bcd2ToByte(lamp_data.addr_bcd>>8)*100 + Bcd2ToByte(lamp_data.addr_bcd & 0x00FF);
	
	LampRunCtrlTable[unit_addr_hex].query_num = MAX_QUERY_NUM;//¸üÐÂÂÖÑ¯´ÎÊý
	
	if((LAMP_DATA_TEMP_SIZE+12+12+6) >= data_size)
	{
		taskENTER_CRITICAL();//½øÈëÁÙ½çÇø
		
		strncpy((char*)LastBuff, (char*)LampDataTempBuff[unit_addr_hex], LAMP_DATA_TEMP_SIZE);//¶Á³öÉÏÒ»´Îµ¥µÆÊý¾Ý
		
		strncpy((char*)LampDataTempBuff[unit_addr_hex], (char*)p+1, 4);//ZigBeeµØÖ·
		strncpy((char*)LampDataTempBuff[unit_addr_hex]+4, (char*)p+9, 34);//µ÷¹âÖµ
		if(data_size == 0x46)
			strncpy((char*)LampDataTempBuff[unit_addr_hex]+38, (char*)p+73, 6);//µÆ¸Ë¶ÔµØµçÑ¹¡¢ÊµÊ±Çã½Ç
		else
		  memset((char*)LampDataTempBuff[unit_addr_hex]+38, 0x30, 6);//µÆ¸Ë¶ÔµØµçÑ¹¡¢ÊµÊ±Çã½Ç
			
		taskEXIT_CRITICAL();//ÍË³öÁÙ½çÇø
	}
	
	last_lamp_data.state = (chr2hex(LastBuff[6])<<4 | chr2hex(LastBuff[7]));//ÉÏÒ»´ÎµÄ×´Ì¬
		
	last_lamp_data.power =  chr2hex(LastBuff[16])<<4;
	last_lamp_data.power = (chr2hex(LastBuff[17])+last_lamp_data.power)<<4;
	last_lamp_data.power = (chr2hex(LastBuff[18])+last_lamp_data.power)<<4;
	last_lamp_data.power =  chr2hex(LastBuff[19])+last_lamp_data.power;
	
	now_power_hex = Bcd2ToByte(lamp_data.power>>8)*100 + Bcd2ToByte(lamp_data.power & 0x00FF);
	last_power_hex = Bcd2ToByte(last_lamp_data.power>>8)*100 + Bcd2ToByte(last_lamp_data.power & 0x00FF);
	
  if((lamp_data.state != last_lamp_data.state) || (abs(now_power_hex - last_power_hex) > 5*10) || (UnitQueryState == PASSIVE_QUERY))
	{
		xQueueSend(GPRSSendAddrQueue, &lamp_data.addr_bcd, configTICK_RATE_HZ*5);
	}
	
	unit_state_ctrl(lamp_data.addr_bcd, lamp_data.state, LampRunCtrlTable[unit_addr_hex].run_state);//ÅÐ¶Ïµ¥µÆÔËÐÐ×´Ì¬ÊÇ·ñÏàÍ¬
	
	ReadRTC_Time(RTC_Format_BCD, &time); //Ê±¼äÐ£Ñé
	buf[0] = hex2chr((time.mon>>4) & 0x0F);
	buf[1] = hex2chr(time.mon & 0x0F);
	buf[2] = hex2chr((time.day>>4) & 0x0F);
	buf[3] = hex2chr(time.day & 0x0F);
	buf[4] = hex2chr((time.week>>4) & 0x0F);
	buf[5] = hex2chr(time.week & 0x0F);
	for(i=0;i<6;i++)
	{
		if(buf[i] != *(p+67+i))
		{
			LampTimeReload();
			break;
		}
	}
	
	NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE + PARAM_TIME_FALG_OFFSET, NorReadBuff, 12);//¶Á³öµÆ²ÎÊý
	if(NorflashDataCheck(NorReadBuff, 12) != EMPTY)
	{
		for(i=0;i<12;i++)
		{
			if(NorReadBuff[i] != *(p+43+i))
			{
				LampParamReload(lamp_data.addr_bcd);
				return;
			}
		}
	
		NorFlashRead(NORFLASH_BALLAST_BASE + unit_addr_hex*NORFLASH_SECTOR_SIZE + STRATEGY_TIME_FALG_OFFSET, NorReadBuff, 12);//¶Á³öµÆ²ßÂÔ
		if(NorflashDataCheck(NorReadBuff, 12) != EMPTY)
		{
			for(i=0;i<12;i++)
			{
				if(NorReadBuff[i] != *(p+55+i))
				{
					LampStrategyReload(lamp_data.addr_bcd);
					return;
				}
			}
		}
  }
}
