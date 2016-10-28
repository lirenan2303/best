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

#define  GSM_BUFF_SIZE      200

SemaphoreHandle_t    GsmTx_semaphore;
static xQueueHandle  GSM_GPRS_queue;
static xQueueHandle  GSM_AT_queue;

volatile ErrorStatus GPRS_ConnectState = ERROR;
WG_ServerParameterType   WG_ServerParameter;
u8 ManagementAddr[MANAGER_ADDR_LENGTH] = {0};

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

const static MessageHandlerMap GPRS_MessageMaps[] =  //��λ����ĳ�ʼ��
{
	{GATEPARAM,      HandleGatewayParam},     /*0x01; ���ز�������*/           
	{LIGHTPARAM,     HandleLightParam},       /*0x02; �Ʋ�������*/  
	{STRATEGY,       HandleStrategyDownload}, /*0x03; �Ʋ�������*/
	{DIMMING,        HandleLightDimmer},      /*0x04; �Ƶ������*/
	{LAMPSWITCH,     HandleLightOnOff},       /*0x05; �ƿ��ؿ���*/
//	{READDATA,       HandleReadBSNData},      /*0x06; ������������*/
//	{DATAQUERY,      HandleGWDataQuery},      /*0x08; �������ݲ�ѯ*/           		    
//	{VERSIONQUERY,   HandleGWVersQuery},      /*0x0C; ����������汾��*/      
//	{SETPARAMLIMIT,  HandleSetParamDog},      /*0x21; ���ù�ǿ�������ʱ���򻮷ֵ����*/
//	{STRATEGYDOWN,   HandleStrategy},         /*0x22; ��������*/
//	{GATEUPGRADE,    HandleGWUpgrade},        /*0x37; ����Զ������*/
//	{TIMEADJUST,     HandleAdjustTime},       /*0x42; Уʱ*/                     
//	{LUXVALUE,       HandleLuxGather},        /*0x43; ���յ����ն�ǿ��ֵ*/		
//	{RESTART,        HandleRestart},          /*0x3F; �豸��λ*/               
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
//	GPIO_SetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
	
  GPIO_InitStructure.GPIO_Pin = PIN_GPRS_PW_EN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIO_GPRS_PW_EN,&GPIO_InitStructure);
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
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־
		
		memcpy(GsmTxData.Buff, buf, buf_size);
		GsmTxData.length = buf_size;
			
		DMA_Cmd(DMA1_Stream3, DISABLE);                         //�ر�DMA���� 
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	    //ȷ��DMA���Ա�����  
		DMA_SetCurrDataCounter(DMA1_Stream3,GsmTxData.length);  //���ݴ�����  
				
		xQueueReceive(GSM_AT_queue, &message, 0);               //����DMA����֮ǰ���AT��Ϣ����
				
		DMA_Cmd(DMA1_Stream3, ENABLE);                          //����DMA���� 
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == SET)
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
		
	  xSemaphoreGiveFromISR(GsmTx_semaphore, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
	}
}

ErrorStatus RelinkTCP(void)
{
	char buf[50];
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	
	sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 10))//���ӷ�����
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 20))
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	return SUCCESS;
}

ErrorStatus SendMsgToSim(char*cmd, char *ack, u32 waittime)
{
	ErrorStatus GSM_MsgState = ERROR;
	portTickType startT = 0;
	char message[sizeof(GsmRxData.Buff)];
	
	startT = xTaskGetTickCount();
  
	while(xTaskGetTickCount() - startT < waittime)
	{
		if(cmd != NULL)
		{
			GsmDMA_TxBuff(cmd, strlen(cmd));
		}
		if(ack == NULL)
		{
			GSM_MsgState = SUCCESS;
			break;
		}
		else
		{
			if(xQueueReceive(GSM_AT_queue, &message, waittime/2) == pdTRUE)
			{
				if(strstr(message, ack) != NULL)
				{
					if((strcmp(ack, "+CCLK: ") == 0))
					{
						UpdataNetTime(message);
					}
					GSM_MsgState = SUCCESS;
					break;
				}
				else if(strstr((char*)message, "*PSUTTZ: ") != NULL)
				{
					NetTimeCentury(message);
				}
			}
		}
	}
	return GSM_MsgState;
}

void SimSendData(u8 *buf,u8 buf_size)
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
}

ErrorStatus SwitchToCommand(void)
{
	vTaskDelay(configTICK_RATE_HZ);
	if(!SendMsgToSim("+++", "OK", configTICK_RATE_HZ * 8))
	{
		printf_str("����ģʽ�л�ʧ��\r");
		return ERROR;
	}
	vTaskDelay(configTICK_RATE_HZ);
	return SUCCESS;
}

ErrorStatus SwitchToData(void)
{
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
		
		if(!SendMsgToSim(NULL, "NORMAL POWER DOWN", configTICK_RATE_HZ * 10))//GSM�ر�����
		{
			continue;
		}
		
		vTaskDelay(configTICK_RATE_HZ);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ );
		GPIO_SetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ * 3 / 2);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		
		if(SendMsgToSim(NULL, "SMS Ready", configTICK_RATE_HZ * 30))//�ȴ�SMS׼�����
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
	
	if(!SendMsgToSim("AT+CIPCCFG=5,2,1024,1\r\n", "\r\nOK\r\n", configTICK_RATE_HZ)) //���ô������
	{
	  printf_str("\r\nGSM������������쳣��\r\n");
		return ERROR;
	}
	
	
	if(!SendMsgToSim("AT+CLTS=1\r\n", "OK", configTICK_RATE_HZ)) //��ȡ����ʱ���
	{
	  printf_str("\r\nGSM��ȡʱ���쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CPIN?\r\n", "+CPIN: READY", configTICK_RATE_HZ * 4))//��ѯSIM���Ƿ�READY
	{
		printf_str("\r\nSIM��READY�쳣��\r\n");
		return ERROR;
	}
	
	vTaskDelay(configTICK_RATE_HZ );	
	
	if(!SendMsgToSim("AT+CREG?\r\n", "+CREG: 0,1", configTICK_RATE_HZ))//��ѯGSM����ע����Ϣ
	{
		printf_str("\r\n����ע����Ϣ�쳣��\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CGATT=0\r", "OK", configTICK_RATE_HZ * 4)) //gprs����
	{
			printf("\r\nGPRS�����쳣\r\n");
			return ERROR;
  }
	
	vTaskDelay(configTICK_RATE_HZ * 5);
	
	if(!SendMsgToSim("AT+CGATT=1\r\n", "OK", configTICK_RATE_HZ * 15))//����TCP����
	{
		printf_str("\r\nGPRS����״̬�쳣��\r\n");
		return ERROR;
	}
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	
	sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 20))//���ӷ�����
	{
		printf_str("\r\nTCP��ʽ����ȷ\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 30))
	{
		printf_str("\r\n�޷����ӷ�����IP�˿�\r\n");
		return ERROR;
	}
	
	vTaskDelay(configTICK_RATE_HZ);
	
	SwitchToCommand();
	
	if(!SendMsgToSim("AT+CCLK?\r\n", "+CCLK: ", configTICK_RATE_HZ)) //��������ʱ��
	{
		printf("��ȡ����ʱ��ʧ��\r\n");
		return ERROR;
  }
	
	SwitchToData();
	
	return SUCCESS;
}

static void HeartRemainTCPConnect(void)
{
	u8 buf[20];
	u8 *p = buf;
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_MANAGER_ADDR, (u16 *)&ManagementAddr, (MANAGER_ADDR_LENGTH + 1)/ 2);
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
	u8 protocol_type;
	portTickType lastT = 0;
	
	for(;;)
	{
		while(!GsmStartConnect());
		GPRS_ConnectState = SUCCESS;

		break;
	}
	
	for(;;)
	{
		if(xQueueReceive(GSM_GPRS_queue, message, configTICK_RATE_HZ/10) == pdTRUE)
		{
			protocol_type = (chr2hex(message[11])<<4 | chr2hex(message[12]));
			const MessageHandlerMap *map = GPRS_MessageMaps;
			for(; map->type != PROTOCOL_NULL; ++map)
			{
				if (protocol_type == map->type) 
				{
					map->handlerFunc(message);
					break;
				}
			}
		}
    if((xTaskGetTickCount() - lastT) >= 30*configTICK_RATE_HZ)
		{
			HeartRemainTCPConnect();
			lastT = xTaskGetTickCount();
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
	xTaskCreate(vGSMTask, "GSMTask", GSM_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 8, NULL);
}

/*******************************END OF FILE************************************/
