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

const static MessageHandlerMap Electric_MessageMaps[] =  //∂˛Œª ˝◊Èµƒ≥ı ºªØ
{
	{GATEPARAM,      HandleGatewayParam},     /*0x01; Õ¯πÿ≤Œ ˝œ¬‘ÿ*/           
//	{LIGHTPARAM,     HandleLightParam},       /*0x02; µ∆≤Œ ˝œ¬‘ÿ*/              
//	{DIMMING,        HandleLightDimmer},      /*0x04; µ∆µ˜π‚øÿ÷∆*/
//	{LAMPSWITCH,     HandleLightOnOff},       /*0x05; µ∆ø™πÿøÿ÷∆*/
//	{READDATA,       HandleReadBSNData},      /*0x06; ∂¡’Ú¡˜∆˜ ˝æ›*/
//	{DATAQUERY,      HandleGWDataQuery},      /*0x08; Õ¯πÿ ˝æ›≤È—Ø*/           		    
//	{VERSIONQUERY,   HandleGWVersQuery},      /*0x0C; ≤ÈÕ¯πÿ»Ìº˛∞Ê±æ∫≈*/      
//	{SETPARAMLIMIT,  HandleSetParamDog},      /*0x21; …Ë÷√π‚«ø∂»«¯”Ú∫Õ ±º‰”ÚªÆ∑÷µ„≤Œ ˝*/
//	{STRATEGYDOWN,   HandleStrategy},         /*0x22; ≤ﬂ¬‘œ¬‘ÿ*/
//	{GATEUPGRADE,    HandleGWUpgrade},        /*0x37; Õ¯πÿ‘∂≥Ã…˝º∂*/
//	{TIMEADJUST,     HandleAdjustTime},       /*0x42; –£ ±*/                     
//	{LUXVALUE,       HandleLuxGather},        /*0x43; Ω” ’µΩπ‚’’∂»«ø∂»÷µ*/		
//	{RESTART,        HandleRestart},          /*0x3F; …Ë±∏∏¥Œª*/               
};


ElectricMessage EleTxData;
ElectricMessage EleRxData;

static void ElectricUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/***************** πƒ‹IOø⁄∫Õ¥Æø⁄ ±÷”*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
	/*****************¥Æø⁄∂‘”¶“˝Ω≈∏¥”√”≥…‰****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	/********************USART∂Àø⁄≈‰÷√********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART≥ı ºªØ…Ë÷√********************/
	USART_InitStructure.USART_BaudRate = 9600;//≤®Ãÿ¬ …Ë÷√
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//◊÷≥§Œ™8Œª ˝æ›∏Ò Ω
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//“ª∏ˆÕ£÷πŒª
	USART_InitStructure.USART_Parity = USART_Parity_No;//Œﬁ∆Ê≈º–£—ÈŒª
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ”≤º˛ ˝æ›¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// ’∑¢ƒ£ Ω
  USART_Init(USART1, &USART_InitStructure); //≥ı ºªØ¥Æø⁄
	
  USART_Cmd(USART1, ENABLE);  // πƒ‹¥Æø⁄
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ø™∆Ùœ‡πÿ÷–∂œ

	/*****************Usart1 NVIC≈‰÷√***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//¥Æø⁄÷–∂œÕ®µ¿
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//«¿’º”≈œ»º∂3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//◊””≈œ»º∂3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÕ®µ¿ πƒ‹
	NVIC_Init(&NVIC_InitStructure);	//∏˘æ›÷∏∂®µƒ≤Œ ˝≥ı ºªØVICºƒ¥Ê∆˜°	
}

static void Electric_TX_DMA_Init(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1 ±÷” πƒ‹ 
  DMA_DeInit(DMA2_Stream7);//DMA1 ˝æ›¡˜4
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//µ»¥˝DMAø…≈‰÷√ 
	
  /* ≈‰÷√ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Õ®µ¿—°‘Ò
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMAÕ‚…Ëµÿ÷∑
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&EleTxData.Buff;//DMA ¥Ê¥¢∆˜0µÿ÷∑
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//¥Ê¥¢∆˜µΩÕ‚…Ëƒ£ Ω
  DMA_InitStructure.DMA_BufferSize = 0x00;// ˝æ›¥´ ‰¡ø
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//Õ‚…Ë∑«‘ˆ¡øƒ£ Ω
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//¥Ê¥¢∆˜‘ˆ¡øƒ£ Ω
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//Õ‚…Ë ˝æ›≥§∂»:8Œª
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//¥Ê¥¢∆˜ ˝æ›≥§∂»:8Œª
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//  π”√∆’Õ®ƒ£ Ω 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//÷–µ»”≈œ»º∂
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //÷∏∂® π”√FIFOƒ£ Ωªπ «÷±Ω”ƒ£ Ω        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//÷∆∂®¡ÀFIFO„–÷µ
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//¥Ê¥¢∆˜Õª∑¢µ•¥Œ¥´ ‰
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//Õ‚…ËÕª∑¢µ•¥Œ¥´ ‰
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//≥ı ºªØDMA Stream
		
  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  // πƒ‹¥Æø⁄3µƒDMA∑¢ÀÕ
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//¥Æø⁄÷–∂œÕ®µ¿
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//«¿’º”≈œ»º∂
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//◊””≈œ»º∂
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÕ®µ¿ πƒ‹
	NVIC_Init(&NVIC_InitStructure);	//∏˘æ›÷∏∂®µƒ≤Œ ˝≥ı ºªØVICºƒ¥Ê∆˜°
}

static void EleRxTxDataInit(void)
{
  EleRxData.index = 0;
	EleRxData.length = 0;
	memset(EleRxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff∏≥÷µŒ™0

  EleTxData.index = 0;
	EleTxData.length = 0;
	memset(EleTxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff∏≥÷µŒ™0
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

void DMA2_Stream7_IRQHandler(void)//GSM_DMA∑¢ÀÕ÷–∂œ
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
		
		DMA_Cmd(DMA2_Stream7, DISABLE);                         //πÿ±’DMA¥´ ‰ 
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	    //»∑±£DMAø…“‘±ª…Ë÷√  
		DMA_SetCurrDataCounter(DMA2_Stream7,EleTxData.length);  // ˝æ›¥´ ‰¡ø  
		DMA_Cmd(DMA2_Stream7, ENABLE);                          //ø™∆ÙDMA¥´ ‰ 
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
