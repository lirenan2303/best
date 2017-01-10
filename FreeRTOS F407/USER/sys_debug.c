#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "misc.h"
#include "sys_debug.h"
#include "norflash.h"
#include "table_process.h"
#include "gsm.h"
#include "rtc.h"
#include "common.h"
#include <stdlib.h>

#define DEBUG_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 1024*2)

#define DEBUG_BUFF_SIZE  50
#define CONFIG_TIME_MAX  30*60*1000/portTICK_RATE_MS

static xQueueHandle  uartDebugQueue;
static SemaphoreHandle_t printf_semaphore;

extern WG_ServerParameterType   WG_ServerParameter;
extern LampBuffType LampAddr;
extern  const char Sofeware_Version[];


volatile u8 DebugMode = 0;

const char *EnterTestMode = "fitbright\r\n";//����ģʽ
const char *FactoryPassword = "FitBright\r\n";//����ģʽ
	
typedef struct
{
	u8 index;
	u8 length;
  char Buff[DEBUG_BUFF_SIZE];
}DebugMessage;

DebugMessage DebugRxData;
u8 TestModeFlag = 0;


static inline void DebugRxDataInput(DebugMessage *temp, char dat)
{
  if(temp->length < DEBUG_BUFF_SIZE)
  {
    temp->Buff[temp->index] = dat;
    temp->index++;
    temp->length++;
  }
}

void printf_str(char *str)
{
	u8 i;
	if(xSemaphoreTake(printf_semaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
		RS485_SEND_SEL
		for(i=0;i<100;i++);
		printf("%s",str);
		for(i=0;i<100;i++);
		RS485_RECEIVE_SEL
		
		xSemaphoreGive(printf_semaphore);
	}
}

void printf_buff(u8 *buf, u16 buf_size)
{
	u8 i;
	
	if(xSemaphoreTake(printf_semaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
		RS485_SEND_SEL
		for(i=0;i<100;i++);
		printf("%.*s", buf_size, buf); 
		if(buf_size != 1)
		{
			printf("\r\n");
		}
		for(i=0;i<100;i++);
		RS485_RECEIVE_SEL
		
		xSemaphoreGive(printf_semaphore);
	}
}

static inline void DebugRxDataClear(DebugMessage *temp)
{
  memset(temp->Buff, 0, DEBUG_BUFF_SIZE);
	temp->index = 0;
	temp->length = 0;
}

static void DebugRxDataInit(void)
{
	DebugRxData.index = 0;
	DebugRxData.length = 0;
	memset(DebugRxData.Buff, 0, DEBUG_BUFF_SIZE);//Buff��ֵΪ0
}

void USART6_IRQHandler(void) 
{
	u8 data;
	
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		data = USART_ReceiveData(DEBUG_COM);
//		if(data == '\n')
//			return;
		
//		if((TestModeFlag == 1) && (data != '\n'))
//		{
//			printf_buff(&data, 1);
//		}
	  DebugRxDataInput(&DebugRxData,data);
		
		if(data == '\n')
		{
			printf_buff((u8*)DebugRxData.Buff, DebugRxData.length);
			
			xQueueSendFromISR(uartDebugQueue, DebugRxData.Buff, &xHigherPriorityTaskWoken);
      portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
			DebugRxDataClear(&DebugRxData);
		}
	}
}

static void SetZone(void)
{
	char buf[50]={0};
	int data;
	u32 Zone_value[2]={0};
	char message[DEBUG_BUFF_SIZE];
	
  printf_str("\r\n����������ʱ��:(����: +08��-05, ����8������5��)");
  printf_str("\r\n>");
	
	if(xQueueReceive(uartDebugQueue,message, CONFIG_TIME_MAX) == pdTRUE)
	{
		if(sscanf(message,"%c%d%c", &buf[0],&data,&buf[1]) == 3)
		{
			if(((buf[0] == '+') || (buf[0] == '-')) && (data < 12) && (buf[1] == '\r'))
			{
        sscanf(message, "%[^\n]", (char*)Zone_value);
//				strcpy((char*)Zone_value, message);
				PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ�������
				RTC_WriteBackupRegister(ZONE_BACKUP, Zone_value[0]);	//ʱ������
				
				memset(Zone_value, 0, sizeof(Zone_value));
				Zone_value[0] = RTC_ReadBackupRegister(ZONE_BACKUP);
				sprintf(buf,"\r\n����ʱ�����óɹ��� (ʱ��Ϊ: %s)\r\n", (char*)Zone_value);
				printf_str(buf);
				return;
			}
			else
			{
				printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
				return;
			}
		}
		else
		{
			printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
			return;
		}
	}
	else
		NVIC_SystemReset();
}

static void SetWGAddr(void)
{
	char message[DEBUG_BUFF_SIZE];
	ErrorStatus state;
	u16 readbuff[15]={0};
	char buf[50];
	u8 i;
	
	printf_str("\r\n���������ص�ַ:(���磺0551010001)");
	printf_str("\r\n>");
	
	if(xQueueReceive(uartDebugQueue,message, CONFIG_TIME_MAX) == pdTRUE)
	{
		if(strlen(message) == 12)
		{
			for(i=0;i<10;i++)
			{
				if(message[i] >= '0' && message[i] <= '9')
					state = SUCCESS;
				else
					state = ERROR;
			}
			if(state == SUCCESS)
			{
				NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, (u16*)message, (MANAGER_ADDR_LENGTH + 1) / 2);
			
				NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, readbuff, (MANAGER_ADDR_LENGTH + 1) / 2);
				sprintf(buf,"\r\n���ص�ַ���óɹ��� (��ַΪ: %s)\r\n", (char*)readbuff);
				printf_str(buf);
				return;
			}
			else 
			{
			  printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
			  return;
			}
		}
		else 
		{
			printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
			return;
		}
	}
	else
		NVIC_SystemReset();
}

static void SetSeriveIPPort(void)
{
	WG_ServerParameterType   DebugWG_ServerPara;
	char message[DEBUG_BUFF_SIZE];
	char buff[30] = {0};
	char endchar = 0;
	short buf[4] = {0};
	int port = 0;
	
	printf_str("\r\n������Զ������IP��ַ���˿ں�(����61.190.38.46,30001;)��");
	printf_str("\r\n>");
	if(xQueueReceive(uartDebugQueue,message, CONFIG_TIME_MAX) == pdTRUE)
	{
		if(6 == sscanf(message,"%hd.%hd.%hd.%hd,%d%c",&buf[0],&buf[1],&buf[2],&buf[3],&port,&endchar))
		{
			if((0<=buf[0] && buf[0]<=255) && (0<=buf[1] && buf[1]<=255) && 
				 (0<=buf[2] && buf[2]<=255) && (0<=buf[3] && buf[3]<=255) &&
				 (0<=port   && port<=65535) && (endchar == ';'))
			{
				sscanf(message, "%[^,]", DebugWG_ServerPara.serverIP);//�洢IP��ַ���˿ڰ����ַ����ʹ洢
				sscanf(message, "%*[^,],%[^;]", DebugWG_ServerPara.serverPORT); 
				strncpy(buff, (char*)DebugWG_ServerPara.serverIP, sizeof(DebugWG_ServerPara.serverIP));
				strncpy(buff+16, (char*)DebugWG_ServerPara.serverPORT, sizeof(DebugWG_ServerPara.serverPORT));
				
				NorFlashWrite(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16*)buff, sizeof(buff)/2);
				
				NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
				sprintf(buff,"\r\nԶ������IP���˿����óɹ��� (IP :%s  �˿� :%s)\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
				printf_str(buff);
				return;
			}
			else 
			{
				printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
				return;
			}
		}
		else 
		{
			printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
			return;
		}
	}
	else
		NVIC_SystemReset();
}

static void SysRunInfor(void)
{
	
}

ErrorStatus NorFlashCheck(u16 data)
{
	char buf[50]={0};
	u8 index;
	u32 i;
	u16 read_data;
	u16 write_buf[1024];
	
  memset(write_buf, data, sizeof(write_buf));
	
	printf_str("\r\nNorFlashд����ȣ�");
	for(i=0;i<4*1024;i++)
	{
		FSMC_NOR_WriteBuffer(write_buf, i*1024, 1024);
		
		if(i%1024 == 0)
		{
			index = i*100/(4*1024);
			buf[0] = hex2chr(ByteToBcd2(index)>>4);
			buf[1] = hex2chr(ByteToBcd2(index)&0x0f);
			printf_str(buf);
			printf_str("% ");
		}
	}
	printf_str("100% ��ɣ�");
	
	printf_str("\r\nNorFlash�����ȣ�");
	for(i=0;i<2*1024*1024;i++)
	{
		FSMC_NOR_ReadBuffer(&read_data, i*2, 1);
		if((i*2)%(1024*1024) == 0)
		{
			index = i*100/(2*1024*1024);
			buf[0] = hex2chr(ByteToBcd2(index)>>4);
			buf[1] = hex2chr(ByteToBcd2(index)&0x0f);
			printf_str(buf);
			printf_str("% ");
		}
		if(read_data != data)
		{
			sprintf(buf,"\r\nNorFlash���󣬴����ַ��0x%x", Bank1_NOR2_ADDR+i*2);
			printf_str(buf);
			
			return ERROR;
		}
	}
	printf_str("100% ��ɣ�");
	
	return SUCCESS;
}

static void FactoryDefault(void)
{
	char message[DEBUG_BUFF_SIZE];
	
	printf_str("\r\n���棺�ָ��������û�ɾ�����в���������ɾ�����������룬����������˳���");
	printf_str("\r\n>");
	
	if(xQueueReceive(uartDebugQueue,message, CONFIG_TIME_MAX) == pdTRUE)
	{
    if(strcmp(message, FactoryPassword) == 0)
		{
			printf_str("\r\nNorFlash��鿪ʼ��������ֹ��");
			
			NorFlashEraseChip();
			printf_str("\r\n\r\n(WR 0x5555)...");
			if(NorFlashCheck(0x5555) == ERROR)
			{
				printf_str("\r\nNorFlashӲ�����ϣ�\r\n");
			  return;
			}
			
      NorFlashEraseChip();
			printf_str("\r\n\r\n(WR 0xAAAA)...");
			if(NorFlashCheck(0xAAAA) == ERROR)
			{
				printf_str("\r\nNorFlashӲ�����ϣ�\r\n");
			  return;
			}
			NorFlashEraseChip();
			
			PWR_BackupAccessCmd(ENABLE);	
			RTC_WriteBackupRegister(ZONE_BACKUP, 0);	
			RTC_WriteBackupRegister(UP_DATE_BACKUP, 0);	
			RTC_WriteBackupRegister(UP_TIME_BACKUP, 0);	
			
			printf_str("\r\nNorFlash���ɹ���Ӳ��״̬������");	
			printf_str("\r\n�����ѻָ��������ã�\r\n");
		}
		else
		{
		  printf_str("\r\n�������������˳��ָ��������ã�\r\n");
			return;
		}
	}
	else
		NVIC_SystemReset();
}

static void WG_ResetInfor(void)
{
	
}

static void Exit(void)
{
	NVIC_SystemReset();
}

static void GatewaySetDebug(void)
{
	char message[DEBUG_BUFF_SIZE];
	
	while(1)
	{
		printf_str("\r\n��ѡ�����ò�����");
		printf_str("\r\n    1.��������ʱ��     2.��������ʱ��    3.�������ص�ַ    4.��������IP���˿� ");
		printf_str("\r\n    5.�Ӵ�������       6.����������      7.ͨ��ʧ�ܵ���    8.���Ʋ�����ѯ ");
		printf_str("\r\n    9.���Ʋ��Բ�ѯ     A.ϵͳ��Ϣ��ѯ    F.�ָ���������    L.�������ݲ�ѯ ");
		printf_str("\r\n    Z.ZigBeeģ����Ϣ   R.���ظ�λ��Ϣ    E.�˳� \r\n> ");
	
		if(xQueueReceive(uartDebugQueue,message,CONFIG_TIME_MAX) == pdTRUE)
		{
			if(message[1] == '\r')
			{
				switch(message[0])
				{
					case '2':   SetZone();         break;
					case '3':   SetWGAddr();       break;
					case '4':   SetSeriveIPPort(); break;
					case 'A':   
					case 'a':   SysRunInfor();     break;
					case 'F':   
					case 'f':   FactoryDefault();  break;
					case 'R':   
					case 'r':   WG_ResetInfor();   break;
					case 'E':   
					case 'e':   Exit();            break;
					
					default:    printf_str("\r\n ��ѡ���������ܣ�\r\n");
				}
		  }
			else 
			{
				printf_str("\r\n�����ʽ����������ѡ��Ҫ���õĲ�����\r\n");
			}
		}
	}
}

void WG_ParamInform(void)
{
	char buf[50];
	u8 time_invl;
	TimeTypeDef time;
	u16 readbuff[15]={0};
	float jd,wd;
	u32 Zone_value[2]={0};
	
	ReadRTC_Time(RTC_Format_BIN, &time);
	printf_str("\r\n/************  COPYRIGHT  2016  DMKJ  *************/");
	printf_str( "\r\n��������           :�������");
	
	sscanf(Sofeware_Version, "%*[^:]:%s", buf);
	printf_str( "\r\n��������汾       :");
	printf_str(buf);
	
	sprintf(buf,"\r\n���ص�ǰʱ��       :%d-%02d-%02d  %02d:%02d:%02d", time.year, time.mon, time.day, time.hour, time.min, time.sec);
	printf_str(buf);
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, readbuff, (MANAGER_ADDR_LENGTH + 1) / 2);
	sprintf(buf,"\r\n���ص�ַ           :%s", (char*)readbuff);
	printf_str(buf);
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	sprintf(buf,"\r\nԶ������IP���˿�   :IP :%s  �˿� :%s", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	printf_str(buf);
	
	Zone_value[0] = RTC_ReadBackupRegister(ZONE_BACKUP);
	NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_LNG_OFFSET, readbuff, 10);
	if(NorflashDataCheck(readbuff ,10) != EMPTY)
	{
		memset(buf, 0, sizeof(buf));
		ConvertToByte(readbuff, 10, (u8 *)buf);
		jd = atoi((const char *)buf);
		jd = jd/1000000;
		NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_LAT_OFFSET, readbuff, 10);
		if(NorflashDataCheck(readbuff ,10) != EMPTY)
		{
			memset(buf, 0, sizeof(buf));
			ConvertToByte(readbuff, 10, (u8 *)buf);
			wd = atoi((const char *)buf);
			wd = wd/1000000;
			sprintf(buf,"\r\nGPS-��γ��-ʱ��    :���� :%f  γ�� :%f  ʱ��:%s", jd, wd, (char*)Zone_value);
			printf_str(buf);
		}
		else
		{
			sprintf(buf,"\r\nGPS-��γ��-ʱ��    :���� :%f  γ�� :��  ʱ��:%s", jd, (char*)Zone_value);
			printf_str(buf);
		}
	}
	else
	{
		sprintf(buf,"\r\nGPS-��γ��-ʱ��    :���� :��  γ�� :��  ʱ��:%s", (char*)Zone_value);
		printf_str(buf);
	}
	
	memset(buf, 0, sizeof(buf));
  NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_DATA_UPDATA_INVL_OPPSET, readbuff, 2);
	if((readbuff[0] == 0xFFFF) || (readbuff[1] == 0xFFFF))
	{
	  sprintf(buf,"\r\n�����ϴ�ʱ����   :�ޣ�");
	}
	else
	{
	  time_invl = (chr2hex(readbuff[0])<<4) + chr2hex(readbuff[1]);
	  sprintf(buf,"\r\n�����ϴ�ʱ����   :%d(����)", time_invl);
	}
	printf_str(buf);
	
	memset(buf, 0, sizeof(buf));
	sprintf(buf,"\r\n��������           :%d(յ)", LampAddr.num);
	printf_str(buf);
	
	printf_str("\r\n----------------------------------------------------");
}

static void TestModeMain(void)
{
	printf_str("\r\n************�Ϸʴ������ܿƼ��ɷ����޹�˾************");
  WG_ParamInform();
	GatewaySetDebug();
}

extern TaskHandle_t xBallastComm1Task;
extern TaskHandle_t xElectTask;
extern TaskHandle_t xGSMTaskHandle;
extern TaskHandle_t xTimePlanTask;
extern TaskHandle_t xKM_vCtrlTask;

void OtherTaskSuspend(void)
{
	vTaskSuspend(xBallastComm1Task);
	vTaskSuspend(xElectTask);
	vTaskSuspend(xGSMTaskHandle);
	vTaskSuspend(xTimePlanTask);
	vTaskSuspend(xKM_vCtrlTask);
}

void WGConfigCheck(void)
{
	u16 ManagemAddr[10]={0};
	const u16 NorResetDate[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, ManagemAddr, (MANAGER_ADDR_LENGTH + 1) / 2);
	if(strcmp((char*)NorResetDate, (char*)ManagemAddr) == 0)
	{
    OtherTaskSuspend();
	}
}

static void vUartDebugTask(void *parameter)
{
  char DebugDateBuff[DEBUG_BUFF_SIZE];
	
	WGConfigCheck();
	
	for(;;)
	{
		if(xQueueReceive(uartDebugQueue,DebugDateBuff,portMAX_DELAY) == pdTRUE)
		{
			if(strcmp(DebugDateBuff,EnterTestMode) == 0)
			{
//				TestModeFlag = 1;
				vTaskDelay(1);
				OtherTaskSuspend();
				
				TestModeMain();
			}
		}
	}
}

static void UartDebugCreateTask(void) 
{
	printf_semaphore = xSemaphoreCreateMutex();
	uartDebugQueue = xQueueCreate(10, DEBUG_BUFF_SIZE);
  xTaskCreate(vUartDebugTask, "UartDebugTask", DEBUG_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void UartDebugInit(void)
{
  UartDebugHardwareInit();
	DebugRxDataInit();
  UartDebugCreateTask();
}
