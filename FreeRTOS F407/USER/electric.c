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
#include "rtc.h"
#include "common.h"
#include "norflash.h"
#include "gateway_protocol.h"
#include "table_process.h"
#include "sys_debug.h"

#define ELECTRIC_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 1024*1)
#define ELECTRIC_BUFF_SIZE  200

SemaphoreHandle_t    EleTx_semaphore;
static xQueueHandle ElectricQueue;
extern LampBuffType LampAddr;

typedef struct 
{
	u8 index;
	u8 length;
  u8 Buff[ELECTRIC_BUFF_SIZE];
}ElectricMessage;

const static ElecMessageHandlerMap Electric_MessageMaps[] =  //¶þÎ»Êý×éµÄ³õÊ¼»¯
{
	{ELECQUERYACK,      ElecHandleGWDataQuery},       /*0x08; Íø¹ØÊý¾Ý²éÑ¯*/
	{ELECSOFTQUERYACK,  ElecHandleSoftVerQuery},      /*0x0E; ²éµçÁ¿²É¼¯Èí¼þ°æ±¾ºÅ*/
	{ELEFTPUPDATAACK,   ElecHandleFTPUpdata},         /*0x1E; µçÁ¿²É¼¯Ä£¿éÔ¶³ÌÉý¼¶*/	                 
};


ElectricMessage EleTxData;
ElectricMessage EleRxData;

static void ElectricUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************Ê¹ÄÜIO¿ÚºÍ´®¿ÚÊ±ÖÓ*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
	/*****************´®¿Ú¶ÔÓ¦Òý½Å¸´ÓÃÓ³Éä****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	/********************USART¶Ë¿ÚÅäÖÃ********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART³õÊ¼»¯ÉèÖÃ********************/
	USART_InitStructure.USART_BaudRate = 9600;//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½
  USART_Init(USART1, &USART_InitStructure); //³õÊ¼»¯´®¿Ú
	
  USART_Cmd(USART1, ENABLE);  //Ê¹ÄÜ´®¿Ú
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//¿ªÆôÏà¹ØÖÐ¶Ï

	/*****************Usart1 NVICÅäÖÃ***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//´®¿ÚÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//ÇÀÕ¼ÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//×ÓÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡	
}

static void Electric_TX_DMA_Init(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1Ê±ÖÓÊ¹ÄÜ 
  DMA_DeInit(DMA2_Stream7);//DMA1Êý¾ÝÁ÷4
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//µÈ´ýDMA¿ÉÅäÖÃ 
	
  /* ÅäÖÃ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Í¨µÀÑ¡Ôñ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMAÍâÉèµØÖ·
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&EleTxData.Buff;//DMA ´æ´¢Æ÷0µØÖ·
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//´æ´¢Æ÷µ½ÍâÉèÄ£Ê½
  DMA_InitStructure.DMA_BufferSize = 0x00;//Êý¾Ý´«ÊäÁ¿
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÍâÉè·ÇÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//´æ´¢Æ÷ÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//´æ´¢Æ÷Êý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// Ê¹ÓÃÆÕÍ¨Ä£Ê½ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//ÖÐµÈÓÅÏÈ¼¶
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //Ö¸¶¨Ê¹ÓÃFIFOÄ£Ê½»¹ÊÇÖ±½ÓÄ£Ê½        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//ÖÆ¶¨ÁËFIFOãÐÖµ
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//´æ´¢Æ÷Í»·¢µ¥´Î´«Êä
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//ÍâÉèÍ»·¢µ¥´Î´«Êä
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//³õÊ¼»¯DMA Stream
		
  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //Ê¹ÄÜ´®¿Ú3µÄDMA·¢ËÍ
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//´®¿ÚÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//×ÓÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡
}

static void EleRxTxDataInit(void)
{
  EleRxData.index = 0;
	EleRxData.length = 0;
	memset(EleRxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff¸³ÖµÎª0

  EleTxData.index = 0;
	EleTxData.length = 0;
	memset(EleTxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff¸³ÖµÎª0
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
		printf_buff(&data, 1);//µ÷ÊÔ
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

void DMA2_Stream7_IRQHandler(void)//GSM_DMA·¢ËÍÖÐ¶Ï
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
		
		DMA_Cmd(DMA2_Stream7, DISABLE);                         //¹Ø±ÕDMA´«Êä 
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	    //È·±£DMA¿ÉÒÔ±»ÉèÖÃ  
		DMA_SetCurrDataCounter(DMA2_Stream7,EleTxData.length);  //Êý¾Ý´«ÊäÁ¿  
		DMA_Cmd(DMA2_Stream7, ENABLE);                          //¿ªÆôDMA´«Êä 
  }
}

static void ElectrolHardwareInit(void)
{
	ElectricUartInit();
	Electric_TX_DMA_Init();
}

void ElecHandleGWDataQuery(u8 *p)
{
	u8 data_size=0;
	u16 WriteBuff[20];
	u16 lamp_num_bcd;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_MANAGER_ID_OFFSET, WriteBuff, 6); 
  ConvertToByte(WriteBuff, 6, p+134);
	
	lamp_num_bcd = ByteToBcd2(LampAddr.num>>8)*256 + ByteToBcd2((u8)LampAddr.num);
	
	*(p+140) = hex2chr((lamp_num_bcd>>12) & 0x000F);
	*(p+141) = hex2chr((lamp_num_bcd>>8) & 0x000F);
	*(p+142) = hex2chr((lamp_num_bcd>>4) & 0x000F);
	*(p+143) = hex2chr((lamp_num_bcd) & 0x000F);
	
	RTC_TimeToChar(WriteBuff);
  ConvertToByte(WriteBuff, 12, p+144);
	
	GPRS_Protocol_Response(ELECQUERYACK, p+15, data_size);
}

void ElecHandleSoftVerQuery(u8 *p)
{
	u8 data_size;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
	GPRS_Protocol_Response(ELECSOFTQUERYACK, p+15, 3);
}

void ElecHandleFTPUpdata(u8 *p)
{
}

static void vElectTask(void *parameter)
{
	u8 message[sizeof(EleRxData.Buff)];
	u8 protocol_type;
	
	for(;;)
	{
		if(xQueueReceive(ElectricQueue, &message, configTICK_RATE_HZ / 10) == pdTRUE)
		{
			protocol_type = (chr2hex(message[11])<<4 | chr2hex(message[12]));
			const ElecMessageHandlerMap *map = Electric_MessageMaps;
			for(; map->type != ELECPROTOCOL_NULL; ++map)
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

TaskHandle_t xElectTask;

void ElectricInit(void)
{
  ElectrolHardwareInit();
	EleRxTxDataInit();
	EleTx_semaphore = xSemaphoreCreateMutex();
  ElectricQueue = xQueueCreate(10, sizeof(EleRxData.Buff));	
	xTaskCreate(vElectTask, "ElectTask", ELECTRIC_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xElectTask);
}
