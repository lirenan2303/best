#ifndef __NORFLASH_H__
#define __NORFLASH_H__

#include <stdbool.h>
#include <stdint.h>
#include "fsmc_nor.h"

/********************************************************************************************************
* NOR_FLASH��ַӳ���
********************************************************************************************************/

//���ص�ַ��������IP���˿ں�
#define NORFLASH_ADDR_BASE                    ((u32)0x00000000)  
#define NORFLASH_MANAGER_ADDR       		    	((u32)0x00001000)
#define NORFLASH_IP1_PORT1    		 	          ((u32)0x00002000)
#define NORFLASH_IP2_PORT2			              ((u32)0x00003000)

//���ز���1
#define NORFLASH_MANAGER_PARA1_BASE		        ((u32)0x00004000)//ASCII��洢*2
#define NORFLASH_MANAGER_ID_OFFSET   	        ((u32)0x00000002)
#define NORFLASH_LNG_OFFSET				            ((u32)0x0000000E)
#define NORFLASH_LAT_OFFSET			    	        ((u32)0x00000022)
#define NORFLASH_ZIGBEE_NUM_OFFSET			      ((u32)0x00000036)
#define NORFLASH_DATA_UPDATA_INVL_OPPSET  	  ((u32)0x00000038)
#define NORFLASH_LOOP_NUM_OFFSET				      ((u32)0x0000003C)
#define NORFLASH_ZIGBEE_CFG_OFFSET	  	      ((u32)0x00000040)

//���ز���2
#define NORFLASH_MANAGER_PARA2_BASE  	        ((u32)0x00005000)//ASCII��洢*2
#define NORFLASH_TURN_ON1_OFFSET              ((u32)0x00000000)
#define NORFLASH_TURN_ON2_OFFSET              ((u32)0x00000001)
#define NORFLASH_TURN_OFF1_OFFSET  		        ((u32)0x00000002)
#define NORFLASH_TURN_OFF2_OFFSET    	        ((u32)0x00000003)

//���ز���3
#define NORFLASH_MANAGER_PARA3_BASE           ((u32)0x00006000)//ASCII��洢*2
#define NORFLASH_VIN_HIGH_OFFSET         		  ((u32)0x00000000)
#define NORFLASH_VIN_LOW_OFFSET         		  ((u32)0x00000006)
#define NORFLASH_NOLOAD_CURRENT_OFFSET        ((u32)0x0000000C)
#define NORFLASH_FULL_LOAD_CURRENT_OFFSET     ((u32)0x00000014)
#define NORFLASH_CNSC_LAMP_OFF_OFFSET         ((u32)0x0000001C)
#define NORFLASH_OTHER_ALARMS_OFFSET          ((u32)0x0000001D)


#define NORFLASH_SECTOR_SIZE   				    ((u32)0x00001000)
//#define NORFLASH_MANAGEM_BASE  				    ((u32)0x00001000)//���ز���-��γ�ȡ��ϴ�����ʱ����
//#define NORFLASH_ONOFFTIME1   				    ((u32)0x00003000)//���ص�ʱ��1
//#define NORFLASH_ONOFFTIME2   				    ((u32)0x00004000)//���ص�ʱ��2
//#define NORFLASH_CHIP_ERASE               ((u32)0x00005000)//'D' 
//#define NORFLASH_MANAGEM_ADDR             ((u32)0x00100000)//���ص�ַ��������IP��ַ���˿ں�
//#define NORFLASH_MANAGEM_TIMEOFFSET 	    ((u32)0x00101000)//���ز���-����ƫ�ơ��ص�ƫ��
//#define NORFLASH_MANAGEM_WARNING   	      ((u32)0x00102000)//���ز���-�澯
//#define NORFLASH_RESET_TIME          			((u32)0x00104000)//����ʱ��
//#define NORFLASH_RESET_COUNT              ((u32)0x00105000)//�������� 
//#define NORFLASH_STRATEGY_BASE 				    ((u32)0x00106000)
//#define NORFLASH_STRATEGY_OK_OFFSET 			((u32)0x00000d00)//����������ʶ
//#define NORFLASH_PARAM_OFFSET   		   		((u32)0x00001000)
//#define NORFLASH_STRATEGY_OFFSET        	((u32)0x00001000)

//����������������������ƫ��  0x64100000 - 0x64800000 ���Դ洢1792յ��������
#define NORFLASH_BALLAST_NUM   				          ((u32)0x00100000)//��������Ŀ
#define NORFLASH_BALLAST_BASE  				          ((u32)0x00101000)//������������ַ
#define PARAM_ZIGBEE_ADDR_OFFSET                ((u32)0x00000000)//(0x00000000-0x00000006)��4*2���ֽ�ASCII��洢ZIGBEE��ַ
#define PARAM_TIME_FALG_OFFSET                  ((u32)0x00000008)//(0x00000008-0x0000001E)��12*2���ֽ�ASCII��洢����ͬ����ʶ
#define PARAM_RATED_POWER_OFFSET                ((u32)0x00000020)//(0x00000020-0x00000026)��4*2���ֽ�ASCII��洢��ƹ���ֵ
#define PARAM_LOOP_NUM_OFFSET                   ((u32)0x0000002A)//(0x0000002A-0x00000030)��4*2���ֽ�ASCII��洢�����Ƹ˺�
#define PARAM_LAMP_TYPE_OFFSET                  ((u32)0x00000028)//(0x00000028-0x00000028)��1*2���ֽ�ASCII��洢������·
#define PARAM_LAMP_POST_NUM_OFFSET              ((u32)0x00000034)//(0x00000034-0x00000034)��1*2���ֽ�ASCII��洢��������
#define PARAM_PORPERTY_OFFSET                   ((u32)0x00000032)//(0x00000032-0x00000032)��1*2���ֽ�ASCII��洢��Դ����
#define PARAM_PHASE_OFFSET                      ((u32)0x00000036)//(0x00000036-0x00000038)��2*2���ֽ�ASCII��洢����Ͷ����
#define STRATEGY_ZIGBEE_ADDR_OFFSET             ((u32)0x00000000)//(0x00000000-0x00000006)��4*2���ֽ�ASCII��洢ZIGBEE��ַ 
#define STRATEGY_TIME_FALG_OFFSET               ((u32)0x00000008)//(0x00000008-0x0000001E)��12*2���ֽ�ASCII��洢����ͬ����ʶ
#define STRATEGY_TYPE_OFFSET                    ((u32)0x00000020)//(0x00000020-0x00000022)��2*2���ֽ�ASCII��洢��������
#define STRATEGY_STAGE_NUM_OFFSET               ((u32)0x00000024)//(0x00000024-0x00000024)��1*2���ֽ�ASCII��洢������� 
#define STRATEGY_FIRST_STATE_OFFSET             ((u32)0x00000026)//(0x00000026-0x00000030)��6*2���ֽ�ASCII��洢���⹦�ʼ�ʱ�� 
#define STRATEGY_SECOND_STATE_OFFSET            ((u32)0x00000032)//(0x00000032-0x0000003C)��6*2���ֽ�ASCII��洢���⹦�ʼ�ʱ��
#define STRATEGY_THIRD_STATE_OFFSET             ((u32)0x0000003E)//(0x0000003E-0x00000048)��6*2���ֽ�ASCII��洢���⹦�ʼ�ʱ��
#define STRATEGY_FOURTH_STATE_OFFSET            ((u32)0x0000004A)//(0x0000004A-0x00000054)��6*2���ֽ�ASCII��洢���⹦�ʼ�ʱ��
#define STRATEGY_FIFTH_STATE_OFFSET             ((u32)0x00000056)//(0x00000056-0x00000060)��6*2���ֽ�ASCII��洢���⹦�ʼ�ʱ��


void NorFlashInit(void);
void NorFlashWrite(u32 addr, const u16 *ram, int len);
void NorFlashEraseParam(uint32_t flash);
void NorFlashRead(u32 addr, u16 *ram, int len);
void NorFlashEraseChip(void);

bool NorFlashMutexLock(uint32_t time);
void NorFlashMutexUnlock(void);
void MemStorage_Convert(u8 *msg, u16 cvt_size, u16 *write_buf);


#endif
