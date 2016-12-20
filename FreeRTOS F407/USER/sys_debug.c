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
#include "gsm.h"

#define DEBUG_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 1024*1)

#define DEBUG_BUFF_SIZE  50

static xQueueHandle  uartDebugQueue;
static SemaphoreHandle_t printf_semaphore;

volatile u8 DebugMode = 0;

const char *TestMode = "fitbright test\r";//����ģʽ
	
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

void printf_data(u8 data)
{
	
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
		if(data == '\n')
			return;
		
		if((TestModeFlag == 1) && (data != '\n'))
		{
			printf_buff(&data, 1);
		}
	  DebugRxDataInput(&DebugRxData,data);
		
		if(data == '\r')
		{
			xQueueSendFromISR(uartDebugQueue, DebugRxData.Buff, &xHigherPriorityTaskWoken);
      portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
			DebugRxDataClear(&DebugRxData);
		}
	}
}

static void SetWGAddr(void)
{
	char message[DEBUG_BUFF_SIZE];
	ErrorStatus state;
	u8 i;
	
	while(1)
	{
	  printf_str("\r\n���������ص�ַ��(���磺0551010001)");
		printf_str("\r\nE.�˳�");
		printf_str("\r\n>");
		if(xQueueReceive(uartDebugQueue,message,portMAX_DELAY) == pdTRUE)
		{
			if(strcmp(message,"E\r") == 0)
			{
			  break;
			}
      else if(strlen(message) == 11)
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
        }
				else 
				{
           printf_str("\r\n������Ϸ������ص�ַ��");
        }
      }
			else 
			{
        printf_str("\r\n������Ϸ������ص�ַ��");
      }
		}
	}
}

static void SetSeriveIPPort(void)
{
	WG_ServerParameterType   DebugWG_ServerPara;
	char message[DEBUG_BUFF_SIZE];
	char buff[30] = {0};
	char endchar = 0;
	short buf[4] = {0};
	int port = 0;
	
	while(1)
	{
	  printf_str("\r\n�����������IP��ַ���˿ں�(����61.190.38.46,30000;)��");
		printf_str("\r\nE.�˳�");
		printf_str("\r\n>");
		if(xQueueReceive(uartDebugQueue,message,portMAX_DELAY) == pdTRUE)
		{
			if(strcmp(message,"E\r") == 0)
			{
			  break;
			}
      else if(6 == sscanf(message,"%hd.%hd.%hd.%hd,%d%c",&buf[0],&buf[1],&buf[2],&buf[3],&port,&endchar))
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
        }
				else 
				{
           printf_str("\r\n������Ϸ���IP��ַ��˿�!");
        }
      }
			else 
			{
        printf_str("\r\n������Ϸ���IP��ַ��˿�!");
      }
		}
	}
}

static void GatewaySetDebug(void)
{
	char message[DEBUG_BUFF_SIZE];
	
	while(1)
	{
		printf_str("\r\n\r\n��ѡ�����ò�����");
		printf_str("\r\n      1.��������ʱ��    2.�������ص�ַ    3.����DNS IP      4.���÷�����IP    5.�������ؾ���");
		printf_str("\r\n      6.��������γ��    7.����������      8.����������      9.������������    A.�Ӵ�������  ");
		printf_str("\r\n      B.������������ѯ  C.���������Բ�ѯ  D.���ز�����ѯ    F.�ָ���������    L.�������ݲ�ѯ");
		printf_str("\r\n      N.���ظ�λ��Ϣ    R.��������        E.����NORFLASH���� \r\n> ");
	
		if(xQueueReceive(uartDebugQueue,message,portMAX_DELAY) == pdTRUE)
		{
			if(strcmp(message,"2\r") == 0)
			{
				SetWGAddr();
			}
			else if(strcmp(message,"4\r") == 0)
			{
				SetSeriveIPPort();
			}
			else 
			{
				printf_str("\r\n �����ʽ����ȷ�����������룡\r\n");
			}
		}
	}
}

static void TestModeMain(void)
{
	printf_str("\r\n\r\n     ����ģʽ");
	printf_str("\r\n�Ϸʴ������ܿƼ��ɷ����޹�˾��");
	printf_str("\r\n��������汾��  ��V");
	printf_str("\r\n���ص�ַ        ��");
	printf_str("\r\n���ص�ǰʱ��    ��");
	printf_str("\r\nĿ�������IP��ַ��");
	printf_str("\r\nĿ��������˿ںţ�");
	
	printf_str("\r\n********************************************************");
	
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
			if(strcmp(DebugDateBuff,TestMode) == 0)
			{
				TestModeFlag = 1;
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
