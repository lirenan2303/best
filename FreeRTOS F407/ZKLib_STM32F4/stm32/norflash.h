#ifndef __NORFLASH_H__
#define __NORFLASH_H__

#include <stdbool.h>
#include <stdint.h>
#include "fsmc_nor.h"

/********************************************************************************************************
* NOR_FLASH地址映射表
********************************************************************************************************/

//网关地址、服务器IP、端口号
#define NORFLASH_ADDR_BASE                    ((u32)0x00000000)  
#define NORFLASH_MANAGER_ADDR       		    	((u32)0x00001000)
#define NORFLASH_IP1_PORT1    		 	          ((u32)0x00002000)
#define NORFLASH_IP2_PORT2			              ((u32)0x00003000)

//网关参数1
#define NORFLASH_MANAGER_PARA1_BASE		        ((u32)0x00004000)//ASCII码存储*2
#define NORFLASH_MANAGER_ID_OFFSET   	        ((u32)0x00000002)
#define NORFLASH_LNG_OFFSET				            ((u32)0x0000000E)
#define NORFLASH_LAT_OFFSET			    	        ((u32)0x00000022)
#define NORFLASH_ZIGBEE_NUM_OFFSET			      ((u32)0x00000036)
#define NORFLASH_DATA_UPDATA_INVL_OPPSET  	  ((u32)0x00000038)
#define NORFLASH_LOOP_NUM_OFFSET				      ((u32)0x0000003C)
#define NORFLASH_ZIGBEE_CFG_OFFSET	  	      ((u32)0x00000040)

//网关参数2
#define NORFLASH_MANAGER_PARA2_BASE  	        ((u32)0x00005000)//ASCII码存储*2
#define NORFLASH_TURN_ON1_OFFSET              ((u32)0x00000000)
#define NORFLASH_TURN_ON2_OFFSET              ((u32)0x00000001)
#define NORFLASH_TURN_OFF1_OFFSET  		        ((u32)0x00000002)
#define NORFLASH_TURN_OFF2_OFFSET    	        ((u32)0x00000003)

//网关参数3
#define NORFLASH_MANAGER_PARA3_BASE           ((u32)0x00006000)//ASCII码存储*2
#define NORFLASH_VIN_HIGH_OFFSET         		  ((u32)0x00000000)
#define NORFLASH_VIN_LOW_OFFSET         		  ((u32)0x00000006)
#define NORFLASH_NOLOAD_CURRENT_OFFSET        ((u32)0x0000000C)
#define NORFLASH_FULL_LOAD_CURRENT_OFFSET     ((u32)0x00000014)
#define NORFLASH_CNSC_LAMP_OFF_OFFSET         ((u32)0x0000001C)
#define NORFLASH_OTHER_ALARMS_OFFSET          ((u32)0x0000001D)


#define NORFLASH_SECTOR_SIZE   				    ((u32)0x00001000)
//#define NORFLASH_MANAGEM_BASE  				    ((u32)0x00001000)//网关参数-经纬度、上传数据时间间隔
//#define NORFLASH_ONOFFTIME1   				    ((u32)0x00003000)//开关灯时间1
//#define NORFLASH_ONOFFTIME2   				    ((u32)0x00004000)//开关灯时间2
//#define NORFLASH_CHIP_ERASE               ((u32)0x00005000)//'D' 
//#define NORFLASH_MANAGEM_ADDR             ((u32)0x00100000)//网关地址、服务器IP地址、端口号
//#define NORFLASH_MANAGEM_TIMEOFFSET 	    ((u32)0x00101000)//网关参数-开灯偏移、关灯偏移
//#define NORFLASH_MANAGEM_WARNING   	      ((u32)0x00102000)//网关参数-告警
//#define NORFLASH_RESET_TIME          			((u32)0x00104000)//重启时间
//#define NORFLASH_RESET_COUNT              ((u32)0x00105000)//重启次数 
//#define NORFLASH_STRATEGY_BASE 				    ((u32)0x00106000)
//#define NORFLASH_STRATEGY_OK_OFFSET 			((u32)0x00000d00)//策略完整标识
//#define NORFLASH_PARAM_OFFSET   		   		((u32)0x00001000)
//#define NORFLASH_STRATEGY_OFFSET        	((u32)0x00001000)

//镇流器参数及策略扇区内偏移  0x64100000 - 0x64800000 可以存储1792盏单灯数据
#define NORFLASH_BALLAST_NUM   				          ((u32)0x00100000)//镇流器数目
#define NORFLASH_BALLAST_BASE  				          ((u32)0x00101000)//镇流器参数基址
#define PARAM_ZIGBEE_ADDR_OFFSET                ((u32)0x00000000)//(0x00000000-0x00000006)共4*2个字节ASCII码存储ZIGBEE地址
#define PARAM_TIME_FALG_OFFSET                  ((u32)0x00000008)//(0x00000008-0x0000001E)共12*2个字节ASCII码存储参数同步标识
#define PARAM_RATED_POWER_OFFSET                ((u32)0x00000020)//(0x00000020-0x00000026)共4*2个字节ASCII码存储标称功率值
#define PARAM_LOOP_NUM_OFFSET                   ((u32)0x0000002A)//(0x0000002A-0x00000030)共4*2个字节ASCII码存储所属灯杆号
#define PARAM_LAMP_TYPE_OFFSET                  ((u32)0x00000028)//(0x00000028-0x00000028)共1*2个字节ASCII码存储所属回路
#define PARAM_LAMP_POST_NUM_OFFSET              ((u32)0x00000034)//(0x00000034-0x00000034)共1*2个字节ASCII码存储负载相线
#define PARAM_PORPERTY_OFFSET                   ((u32)0x00000032)//(0x00000032-0x00000032)共1*2个字节ASCII码存储光源类型
#define PARAM_PHASE_OFFSET                      ((u32)0x00000036)//(0x00000036-0x00000038)共2*2个字节ASCII码存储主辅投属性
#define STRATEGY_ZIGBEE_ADDR_OFFSET             ((u32)0x00000000)//(0x00000000-0x00000006)共4*2个字节ASCII码存储ZIGBEE地址 
#define STRATEGY_TIME_FALG_OFFSET               ((u32)0x00000008)//(0x00000008-0x0000001E)共12*2个字节ASCII码存储策略同步标识
#define STRATEGY_TYPE_OFFSET                    ((u32)0x00000020)//(0x00000020-0x00000022)共2*2个字节ASCII码存储方案类型
#define STRATEGY_STAGE_NUM_OFFSET               ((u32)0x00000024)//(0x00000024-0x00000024)共1*2个字节ASCII码存储调光段数 
#define STRATEGY_FIRST_STATE_OFFSET             ((u32)0x00000026)//(0x00000026-0x00000030)共6*2个字节ASCII码存储调光功率及时间 
#define STRATEGY_SECOND_STATE_OFFSET            ((u32)0x00000032)//(0x00000032-0x0000003C)共6*2个字节ASCII码存储调光功率及时间
#define STRATEGY_THIRD_STATE_OFFSET             ((u32)0x0000003E)//(0x0000003E-0x00000048)共6*2个字节ASCII码存储调光功率及时间
#define STRATEGY_FOURTH_STATE_OFFSET            ((u32)0x0000004A)//(0x0000004A-0x00000054)共6*2个字节ASCII码存储调光功率及时间
#define STRATEGY_FIFTH_STATE_OFFSET             ((u32)0x00000056)//(0x00000056-0x00000060)共6*2个字节ASCII码存储调光功率及时间


void NorFlashInit(void);
void NorFlashWrite(u32 addr, const u16 *ram, int len);
void NorFlashEraseParam(uint32_t flash);
void NorFlashRead(u32 addr, u16 *ram, int len);
void NorFlashEraseChip(void);

bool NorFlashMutexLock(uint32_t time);
void NorFlashMutexUnlock(void);
void MemStorage_Convert(u8 *msg, u16 cvt_size, u16 *write_buf);


#endif
