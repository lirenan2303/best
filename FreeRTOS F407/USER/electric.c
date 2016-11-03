#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "electric.h"
#include "uart_debug.h"
#include "common.h"
#include "gateway_protocol.h"

#define ELECTRIC_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 1024*1)
#define ELECTRIC_BUFF_SIZE  100

xSemaphoreHandle EleTx_semaphore;
static xQueueHandle ElectricQueue;

typedef struct 
{
	u8 index;
	u8 length;
  u8 Buff[100];
}ElectricMessage;

const static MessageHandlerMap Electric_MessageMaps[] =  //��λ����ĳ�ʼ��
{
	{GATEPARAM,      HandleGatewayParam},     /*0x01; ���ز�������*/           
//	{LIGHTPARAM,     HandleLightParam},       /*0x02; �Ʋ�������*/              
//	{DIMMING,        HandleLightDimmer},      /*0x04; �Ƶ������*/
//	{LAMPSWITCH,     HandleLightOnOff},       /*0x05; �ƿ��ؿ���*/
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


ElectricMessage EleTxData;
ElectricMessage EleRxData;

static void ElectricUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************ʹ��IO�ںʹ���ʱ��*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
	/*****************���ڶ�Ӧ���Ÿ���ӳ��****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	/********************USART�˿�����********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART��ʼ������********************/
	USART_InitStructure.USART_BaudRate = 9600;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	/*****************Usart1 NVIC����***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ����	
}

static void Electric_TX_DMA_Init(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1ʱ��ʹ�� 
  DMA_DeInit(DMA2_Stream7);//DMA1������4
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&EleTxData.Buff;//DMA �洢��0��ַ
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
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//��ʼ��DMA Stream
		
  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ����
}

static void EleRxTxDataInit(void)
{
  EleRxData.index = 0;
	EleRxData.length = 0;
	memset(EleRxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff��ֵΪ0

  EleTxData.index = 0;
	EleTxData.length = 0;
	memset(EleTxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff��ֵΪ0
}

static inline void ElectricRxDataInput(ElectricMessage *temp, char dat)
{
  if(temp->length < ELECTRIC_BUFF_SIZE)
  {
    temp->Buff[temp->index] = dat;
    temp->index++;
    temp->length++;
  }
}

static inline void ElectricRxDataClear(ElectricMessage *temp)
{
  memset(temp->Buff, 0, ELECTRIC_BUFF_SIZE);
	temp->index = 0;
	temp->length = 0;
}

void USART1_IRQHandler(void)
{
	unsigned char data;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
    data = USART_ReceiveData(USART1);
	  if(data == 0x02)
		{
			ElectricRxDataClear(&EleRxData);
		}
			
		ElectricRxDataInput(&EleRxData,data);
			
		if(data == 0x03)
		{
			BCC_CheckSum(EleRxData.Buff,EleRxData.length-3);
			xQueueSendFromISR(ElectricQueue, &EleRxData.Buff, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
			ElectricRxDataClear(&EleRxData);
		}
	}
}

void DMA2_Stream7_IRQHandler(void)//GSM_DMA�����ж�
{
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7); 
	DMA_Cmd(DMA2_Stream7, DISABLE);
	xSemaphoreGive(EleTx_semaphore);
}


void EleDMA_TxBuff(char *buf, u8 buf_size)
{
	if(xSemaphoreTake(EleTx_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		memcpy(EleTxData.Buff, buf, buf_size);
		EleTxData.length = buf_size;
		
		DMA_Cmd(DMA2_Stream7, DISABLE);                         //�ر�DMA���� 
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	    //ȷ��DMA���Ա�����  
		DMA_SetCurrDataCounter(DMA2_Stream7,EleTxData.length);  //���ݴ�����  
		DMA_Cmd(DMA2_Stream7, ENABLE);                          //����DMA���� 
  }
}


static void ElectrolHardwareInit(void)
{
	ElectricUartInit();
	Electric_TX_DMA_Init();
}

static void vElectTask(void *parameter)
{
	u8 message[sizeof(EleRxData.Buff)];
	u8 protocol_type;
	
	for(;;)
	{
		if(xQueueReceive(ElectricQueue, &message, configTICK_RATE_HZ / 10) == pdTRUE)
		{
			protocol_type = (chr2hex(message[5])<<4 | chr2hex(message[6]));
			const MessageHandlerMap *map = Electric_MessageMaps;
			for(; map->type != PROTOCOL_NULL; ++map)
			{
				if (protocol_type == map->type) 
				{
					map->handlerFunc(message);
					break;
				}
			}
		}
	}
}


void ElectricInit(void)
{
  ElectrolHardwareInit();
	EleRxTxDataInit();
	vSemaphoreCreateBinary(EleTx_semaphore);
  ElectricQueue = xQueueCreate(8, sizeof(ElectricMessage));	
	xTaskCreate(vElectTask, "ElectTask", ELECTRIC_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}
