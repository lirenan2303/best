#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "gsm.h"
#include <string.h>
#include <stdio.h>
#include "uart_debug.h"
#include "norflash.h"
#include "rtc.h"
#include "gateway_protocol.h"
#include "ballast_protocol.h"
#include "common.h"
#include "sys_debug.h"

#define GSM_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 1024*2)

#define  GSM_COM            USART3

#define  GPIO_GSM           GPIOB
#define  GSM_Tx             GPIO_Pin_10
#define  GSM_Rx             GPIO_Pin_11

#define  GPIO_GPRS_Reset    GPIOC
#define  Pin_GPRS_Reset     GPIO_Pin_2

#define  GPIO_GPRS_PW_EN    GPIOC
#define  PIN_GPRS_PW_EN     GPIO_Pin_0

TaskHandle_t xGSMTaskHandle;
SemaphoreHandle_t    GsmTx_semaphore;
xQueueHandle  GSM_GPRS_queue;
extern xQueueHandle  GPRSSendAddrQueue;
xQueueHandle  GSM_AT_queue;

WG_ServerParameterType   WG_ServerParameter;
static portTickType Heart_lastT = 0;
static portTickType Gsm_sendT = 0;


typedef enum
{
	GPRS_TYPE = 0,
	AT_TYPE,
	NULL_TYPE,
}FrameTypeList;

//typedef enum 
//{
//	TYPE_NONE = 0,
//	TYPE_GPRS_DATA,
//	TYPE_AT_DATA,
//}GsmTaskMessageType;

typedef struct
{
	u8 index;
	u8 length;
  u8 Buff[GSM_BUFF_SIZE];
}GsmMessage;

GsmMessage GsmTxData;
GsmMessage GsmRxData;

FrameTypeList FrameType_Flag;

const static WGMessageHandlerMap GPRS_MessageMaps[] =  //��λ����ĳ�ʼ��
{
	{GATEPARAM,        HandleGatewayParam},     /*0x01; ���ز�������*/           
	{LAMPPARAM,        HandleLampParam},        /*0x02; �Ʋ�������*/  
	{LAMPSTRATEGY,     HandleLampStrategy},     /*0x03; �Ʋ�������*/
	{LAMPDIMMING,      HandleLampDimmer},       /*0x04; �Ƶ������*/
	{LAMPONOFF,        HandleLampOnOff},        /*0x05; �ƿ��ؿ���*/
	{READLAMPDATA,     HandleReadBSNData},      /*0x06; ������������*/
	{BRANCHCTRL,       HandleBranchOnOff},      /*0x07; ���ػ�·����*/
	{DATAQUERY,        HandleGWDataQuery},      /*0x08; �������ݲ�ѯ*/
	{TIMEADJUST,       HandleAdjustTime},       /*0x0B; Уʱ*/
	{VERSIONQUERY,     HandleGWVersQuery},      /*0x0C; ����������汾��*/ 
  {ELECVERSION,      HandleElecVersQuery},    /*0x0E; �����������汾��*/	
	{GWADDRQUERY,      HandleGWAddrQuery},      /*0x11; �����ص�ַ*/
	{SETSERVERIPPORT,  HandleSetIPPort},        /*0x14; ����Ŀ�������IP�Ͷ˿�*/
	{GATEUPGRADE,      HandleGWUpgrade},        /*0x15; ����Զ������*/
	{GPRSQUALITY,      HandleSignalQuality},    /*0x17; gprs�ź�ǿ��*/
	{TUNNELSTRATEGY,   HandleTunnelStrategy},   /*0x22; �����������*/                    
	{RESTART,          HandleRestart},          /*0x3F; �豸��λ*/    
  {WGPROTOCOL_NULL,  NULL},                   /*����*/  	
};
 
static void GSM_USART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************ʹ��IO�ںʹ���ʱ��*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	/*****************���ڶ�Ӧ���Ÿ���ӳ��****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
	/********************USART�˿�����********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART��ʼ������********************/
	USART_InitStructure.USART_BaudRate = 57600;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;//���ͽ���Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(GSM_COM, &USART_InitStructure); //��ʼ������
	
  USART_Cmd(GSM_COM, ENABLE);  //ʹ�ܴ���
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(GSM_COM, USART_IT_RXNE, ENABLE);//��������ж�

	/*****************Usart1 NVIC����***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���� 
}

static void GSM_CtrlPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/****************GSM��Դ����*******************/
	GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
	
  GPIO_InitStructure.GPIO_Pin = PIN_GPRS_PW_EN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIO_GPRS_PW_EN,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Pin_GPRS_Reset;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIO_GPRS_Reset,&GPIO_InitStructure);
}

static void GSM_TX_DMA_Init(void) 
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
  DMA_DeInit(DMA1_Stream3);//DMA1������3
	
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&GsmTxData.Buff;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = 0x00;//���ݴ�����
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //ָ��ʹ��FIFOģʽ����ֱ��ģʽ        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//�ƶ���FIFO��ֵ
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//��ʼ��DMA Stream
	
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ����
}

static inline void GsmRxDataInput(GsmMessage *temp, char dat)
{
  if(temp->length < GSM_BUFF_SIZE)
  {
    temp->Buff[temp->index] = dat;
    temp->index++;
    temp->length++;
  }
}

static inline void GsmRxDataClear(GsmMessage *temp)
{
	temp->index = 0;
	temp->length = 0;
	memset(temp->Buff, 0, GSM_BUFF_SIZE);
}

static void GsmRxTxDataInit(void)
{
	GsmRxData.index = 0;
	GsmRxData.length = 0;
	memset(GsmRxData.Buff, 0, GSM_BUFF_SIZE);//Buff��ֵΪ0
	
	GsmTxData.index = 0;
	GsmTxData.length = 0;
	memset(GsmTxData.Buff, 0, GSM_BUFF_SIZE);//Buff��ֵΪ0
}

static inline void Frame_Judge(GsmMessage *temp, char dat)
{
	if(temp->length == 0x00)
	{
		if(dat == 0x02)
			FrameType_Flag = GPRS_TYPE;
		else if(dat == 0x0D)
			FrameType_Flag = AT_TYPE;
		else 
			FrameType_Flag =  NULL_TYPE;
	}
}

void USART3_IRQHandler(void)
{
	unsigned char data;
	unsigned char BccRecheck;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
    data = USART_ReceiveData(GSM_COM);
		printf_buff(&data, 1);
    Frame_Judge(&GsmRxData,data);
		if(FrameType_Flag == GPRS_TYPE)
		{
			if(data == 0x02)
			{
				GsmRxDataClear(&GsmRxData);
			}
			
		  GsmRxDataInput(&GsmRxData,data);
			
			if(data == 0x03)
			{
				BccRecheck = BCC_CheckSum((unsigned char *)GsmRxData.Buff,GsmRxData.length-3);
				if(GsmRxData.Buff[GsmRxData.index-3] == hex2chr(BccRecheck>>4) &&
					 GsmRxData.Buff[GsmRxData.index-2] == hex2chr(BccRecheck&0x0F))
				{
				  xQueueSendFromISR(GSM_GPRS_queue, GsmRxData.Buff, &xHigherPriorityTaskWoken);
				  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
				}
        GsmRxDataClear(&GsmRxData);
			}
	  }
		else if(FrameType_Flag == AT_TYPE)
		{
			if((GsmRxData.length <= 1) && (data == 0x0D))
			{
				GsmRxDataClear(&GsmRxData);
			}
			
		  GsmRxDataInput(&GsmRxData,data);
		
			if((GsmRxData.length > 4) && (data == 0x0A))
			{
				xQueueSendFromISR(GSM_AT_queue, GsmRxData.Buff, &xHigherPriorityTaskWoken);
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
				GsmRxDataClear(&GsmRxData);
			}
		}
//		else
//		{
//			GsmRxDataClear(&GsmRxData);
//		}
	}
}


void GsmDMA_TxBuff(char *buf, u8 buf_size)
{
	char message[sizeof(GsmRxData.Buff)];
	
	if(xSemaphoreTake(GsmTx_semaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
		while(xTaskGetTickCount() - Gsm_sendT <= 200/portTICK_RATE_MS)
		{
			vTaskDelay(2/portTICK_RATE_MS);
		}
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־
		
		memcpy(GsmTxData.Buff, buf, buf_size);
		GsmTxData.length = buf_size;
			
		DMA_Cmd(DMA1_Stream3, DISABLE);                         //�ر�DMA���� 
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	    //ȷ��DMA���Ա�����  
		DMA_SetCurrDataCounter(DMA1_Stream3,GsmTxData.length);  //���ݴ�����  
				
		xQueueReceive(GSM_AT_queue, &message, 0);               //����DMA����֮ǰ���AT��Ϣ����
				
		DMA_Cmd(DMA1_Stream3, ENABLE);                          //����DMA���� 
		
	}
	else
	{
		NVIC_SystemReset();
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == SET)
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
		
		Gsm_sendT = xTaskGetTickCount();
	  xSemaphoreGiveFromISR(GsmTx_semaphore, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
	}
}

ErrorStatus RelinkTCP(void)
{
	char buf[50];
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	
	sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 5))//���ӷ�����
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 10))
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	return SUCCESS;
}

ErrorStatus SendMsgToSim(char *cmd, char *ack, u32 waittime)//����atָ�gsmģ��
{
	portTickType startT = 0;
	char message[sizeof(GsmRxData.Buff)];
	u8 count=0;

	while(count < 3)
	{
		count ++;
		if(cmd != NULL)
		{
			GsmDMA_TxBuff(cmd, strlen(cmd));
		}
		if(ack == NULL)
		{
			return SUCCESS;
		}
		else
		{
			startT = xTaskGetTickCount();
			while(xTaskGetTickCount() - startT < waittime)
			{
				if(xQueueReceive(GSM_AT_queue, &message, waittime) == pdTRUE)
				{
					if(strstr(message, ack) != NULL)
					{
						if(strcmp(ack, "+CCLK: ") == 0)
						{
							UpdataNetTime(message);
						}
						else if(strstr(message, "+CSQ:") != NULL)
						{
							CSQ_Reply(message);
						}
						return SUCCESS;
					}
					else if(strstr((char*)message, "*PSUTTZ: ") != NULL)
					{
						NetTimeCentury(message);
					}
				}
			}
		}
	}
	return ERROR;
}

void SimSendData(u8 *buf,u8 buf_size)//����gprs���ݵ�������
{
	char message[sizeof(GsmRxData.Buff)];
	
	if(xQueueReceive(GSM_AT_queue, &message, 0) == pdTRUE)
	{
		if(strcmp(message, "\r\nCLOSED\r\n") == 0)
		{
		  if(RelinkTCP() == ERROR)
			{
				if(GsmStartConnect() == ERROR)
					NVIC_SystemReset();
			}
		}
	}
  GsmDMA_TxBuff((char*)buf,buf_size);
	
	Heart_lastT = xTaskGetTickCount();
}

ErrorStatus SwitchToCommand(void)
{
	if(xSemaphoreTake(GsmTx_semaphore, configTICK_RATE_HZ * 5) == pdTRUE)
	{
	  xSemaphoreGive(GsmTx_semaphore);
	}
	
	vTaskDelay(1500/portTICK_RATE_MS);
	if(!SendMsgToSim("+++", "OK", configTICK_RATE_HZ * 5))
	{
		printf_str("����ģʽ�л�ʧ��\r");
		return ERROR;
	}
	return SUCCESS;
}

ErrorStatus SwitchToData(void)
{
	vTaskDelay(500/portTICK_RATE_MS);
	
	if(!SendMsgToSim("ATO\r\n", "CONNECT", configTICK_RATE_HZ * 5))
	{
		printf_str("����ģʽ�л�ʧ��\r");
		return ERROR;
	}
	return SUCCESS;
}

static void GSM_ModuleStart(void)
{
	while(1)
	{
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
	 	vTaskDelay(configTICK_RATE_HZ * 3 / 2);
		GPIO_SetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		
		if(!SendMsgToSim(NULL, "NORMAL POWER DOWN", configTICK_RATE_HZ * 5))//GSM�ر�����
		{
			continue;
		}
		
		vTaskDelay(configTICK_RATE_HZ);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ );
		GPIO_SetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ * 3 / 2);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		
		if(SendMsgToSim(NULL, "SMS Ready", configTICK_RATE_HZ * 15))//�ȴ�SMS׼�����
	  {
			break;
	  }
	}
}

ErrorStatus GsmStartConnect(void)
{
	char buf[50];
	
	GSM_ModuleStart();//GSM�ϵ�����
	
	if(!SendMsgToSim("AT\r\n", "OK", configTICK_RATE_HZ))//���Դ���ͨѶ�Ƿ�����
	{
		printf_str("\r\nGSM����ͨ�Žӿ��쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+IPR=57600\r\n", "OK", configTICK_RATE_HZ))//����ͨ�Ų�����57600
	{
	  printf_str("\r\nGSMͨ�Ų����������쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("ATE0\r\n", "OK", configTICK_RATE_HZ))//����ģʽ�ر�
	{
	  printf_str("\r\n����ģʽ�ر��쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CIPMODE=1\r\n", "OK", configTICK_RATE_HZ)) //ѡ��TCPIPģʽΪ͸������
	{
	  printf_str("\r\n͸�����������쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CIPCCFG=5,2,1024,1\r\n", "\r\nOK\r\n", configTICK_RATE_HZ)) //���ô������   ���200ms
	{
	  printf_str("\r\nGSM������������쳣��\r\n");
		return ERROR;
	}
	
	
	if(!SendMsgToSim("AT+CLTS=1\r\n", "OK", configTICK_RATE_HZ)) //��ȡ����ʱ���
	{
	  printf_str("\r\nGSM��ȡʱ���쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CPIN?\r\n", "+CPIN: READY", configTICK_RATE_HZ * 2))//��ѯSIM���Ƿ�READY
	{
		printf_str("\r\nSIM��READY�쳣��\r\n");
		return ERROR;
	}
	
	vTaskDelay(configTICK_RATE_HZ);	
	
	if(!SendMsgToSim("AT+CREG?\r\n", "+CREG: 0,1", configTICK_RATE_HZ))//��ѯGSM����ע����Ϣ
	{
		printf_str("\r\n����ע����Ϣ�쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CGATT=0\r", "OK", configTICK_RATE_HZ * 2)) //gprs����
	{
		printf("\r\nGPRS�����쳣\r\n");
		return ERROR;
  }
	
	vTaskDelay(configTICK_RATE_HZ * 3);
	
	if(!SendMsgToSim("AT+CGATT=1\r\n", "OK", configTICK_RATE_HZ * 10))//����TCP����
	{
		printf_str("\r\nGPRS����״̬�쳣��\r\n");
		return ERROR;
	}
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	
	sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 10))//���ӷ�����
	{
		printf_str("\r\nTCP�����쳣\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 15))
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	return SUCCESS;
}

static void HeartRemainTCPConnect(void)
{
	u8 buf[20];
	u8 *p = buf;
	u8 ManagementAddr[MANAGER_ADDR_LENGTH];
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, (u16 *)ManagementAddr, (MANAGER_ADDR_LENGTH + 1)/ 2);
	*p = 0x02;
	strncpy((char*)(p+1), (char*)ManagementAddr, MANAGER_ADDR_LENGTH);
	*(p+MANAGER_ADDR_LENGTH+1) = 0x03;
	
  SimSendData(buf,12);
}

static void GSMInitHardware(void)
{
	GSM_USART_Init();
	GSM_CtrlPinInit();
	GSM_TX_DMA_Init();
}

static void vGSMTask(void *parameter)
{
	u8 message[sizeof(GsmRxData.Buff)];
	u8 protocol_type,i;
	u8 num,reset_flag = 0x31;
	u16 addr[10];
  portTickType CSQ_TCP_lastT=0;
	
	for(;;)
	{
		while(!GsmStartConnect());
		
		vTaskDelay(configTICK_RATE_HZ);
		
		if(!SwitchToCommand())
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}
		if(!SendMsgToSim("AT+CCLK?\r\n", "+CCLK: ", configTICK_RATE_HZ))//��������ʱ��
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}
		if(!SendMsgToSim("AT+CSQ\r\n", "+CSQ:", configTICK_RATE_HZ*2))//��ѯ�ź�ǿ��
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}

		GPRS_Protocol_Response(RESTART|0x80, &reset_flag, 1);
		break;
	}
	
	for(;;)
	{
		while(1)
		{
			if(uxQueueMessagesWaiting(GPRSSendAddrQueue) == 0)
				break;
			else
			{
				num=0;
				for(i=0;i<3;i++)//һ����ഫ��3����������86����
				{
					if(xQueueReceive(GPRSSendAddrQueue, &addr[i], 1000/portTICK_RATE_MS) == pdFALSE)//���ȴ�һ��
					{
						GPRSSendUnitDataFun(addr, num, 0x86);
						break;
					}
					else
					{
						num++;
						if(num == 3)
						{
						  GPRSSendUnitDataFun(addr, num, 0x86);
						  break;
						}
					}
				}
			}
		}
		while(1)
		{
			if(uxQueueMessagesWaiting(GPRSSendAddrQueue) != 0)
				break;
			else
			{
				if(xQueueReceive(GSM_GPRS_queue, message, 500/portTICK_RATE_MS) == pdTRUE)
				{
					protocol_type = (chr2hex(message[11])<<4 | chr2hex(message[12]));
					const WGMessageHandlerMap *map = GPRS_MessageMaps;
					for(; map->type != WGPROTOCOL_NULL; ++map)
					{
						if (protocol_type == map->type) 
						{
							map->handlerFunc(message);
							break;
						}
					}
				}
				if((xTaskGetTickCount() - Heart_lastT) >= 30*configTICK_RATE_HZ)
				{
					HeartRemainTCPConnect();
					Heart_lastT = xTaskGetTickCount();
				}
				if((xTaskGetTickCount() - CSQ_TCP_lastT) >= 55*60*configTICK_RATE_HZ)
				{
					if(!SwitchToCommand())
					{
						if(GsmStartConnect() == ERROR)
							NVIC_SystemReset();
					}
					if(!SendMsgToSim("AT+CIPSTATUS\r\n", "STATE: CONNECT OK", configTICK_RATE_HZ))//���TCP����״��
					{
						if(GsmStartConnect() == ERROR)
							NVIC_SystemReset();
					}
					if(!SendMsgToSim("AT+CSQ\r\n", "+CSQ:", configTICK_RATE_HZ*2))//��ѯ�ź�ǿ��
					{
						if(GsmStartConnect() == ERROR)
							NVIC_SystemReset();
					}
					CSQ_TCP_lastT = xTaskGetTickCount();
				}
		  }
	  }
	}
}

void GSMInit(void)
{
  GSMInitHardware();
	GsmRxTxDataInit();
	GsmTx_semaphore = xSemaphoreCreateMutex();
	GSM_GPRS_queue = xQueueCreate(10, sizeof(GsmRxData.Buff));
	GSM_AT_queue = xQueueCreate(1, sizeof(GsmRxData.Buff));
	xTaskCreate(vGSMTask, "GSMTask", GSM_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &xGSMTaskHandle);
}

/*******************************END OF FILE************************************/
