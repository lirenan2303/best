#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include "ballast_comm.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "uart_debug.h"
#include "common.h"
#include "ballast_protocol.h"
#include "gateway_protocol.h"


#define ZIGBEE_TASK_STACK_SIZE		     (configMINIMAL_STACK_SIZE + 1024)
#define BALLAST_BUFF_SIZE   100

#define  BALLAST_COMM1      UART4

xSemaphoreHandle ballastComm1Tx_semaphore;
static xQueueHandle  BallastComm1Queue;

typedef struct
{
	u8 index;
	u8 length;
  u8 Buff[BALLAST_BUFF_SIZE];
}BallastMessage;


const static MessageHandlerMap Ballast_MessageMaps[] =  //∂˛Œª ˝◊Èµƒ≥ı ºªØ
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

BallastMessage BallastComm1RxData;
BallastMessage BallastComm1TxData;

static void BallastUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/***************** πƒ‹IOø⁄∫Õ¥Æø⁄ ±÷”*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
 
	/*****************¥Æø⁄∂‘”¶“˝Ω≈∏¥”√”≥…‰****************/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	/********************USART∂Àø⁄≈‰÷√********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   /*****************USART≥ı ºªØ…Ë÷√********************/
	USART_InitStructure.USART_BaudRate = 38400;//≤®Ãÿ¬ …Ë÷√
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//◊÷≥§Œ™8Œª ˝æ›∏Ò Ω
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//“ª∏ˆÕ£÷πŒª
	USART_InitStructure.USART_Parity = USART_Parity_No;//Œﬁ∆Ê≈º–£—ÈŒª
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Œﬁ”≤º˛ ˝æ›¡˜øÿ÷∆
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// ’∑¢ƒ£ Ω
  USART_Init(UART4, &USART_InitStructure); //≥ı ºªØ¥Æø⁄
	
  USART_Cmd(UART4, ENABLE);  // πƒ‹¥Æø⁄
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//ø™∆Ùœ‡πÿ÷–∂œ

	/*****************Usart1 NVIC≈‰÷√***********************/
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//¥Æø⁄÷–∂œÕ®µ¿
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//«¿’º”≈œ»º∂15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//◊””≈œ»º∂3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÕ®µ¿ πƒ‹
	NVIC_Init(&NVIC_InitStructure);	//∏˘æ›÷∏∂®µƒ≤Œ ˝≥ı ºªØVICºƒ¥Ê∆˜°
}

static void Ballast_TX_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1 ±÷” πƒ‹ 
  DMA_DeInit(DMA1_Stream4);//DMA1 ˝æ›¡˜4
	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//µ»¥˝DMAø…≈‰÷√ 
	
  /* ≈‰÷√ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Õ®µ¿—°‘Ò
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMAÕ‚…Ëµÿ÷∑
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&BallastComm1RxData.Buff;//DMA ¥Ê¥¢∆˜0µÿ÷∑
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
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//≥ı ºªØDMA Stream
		
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  // πƒ‹¥Æø⁄4µƒDMA∑¢ÀÕ
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//¥Æø⁄÷–∂œÕ®µ¿
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//«¿’º”≈œ»º∂
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//◊””≈œ»º∂
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÕ®µ¿ πƒ‹
	NVIC_Init(&NVIC_InitStructure);	//∏˘æ›÷∏∂®µƒ≤Œ ˝≥ı ºªØVICºƒ¥Ê∆˜°
}

static void BallastComm1RxTxDataInit(void)
{
  BallastComm1RxData.index = 0;
	BallastComm1RxData.length = 0;
	memset(BallastComm1RxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff∏≥÷µŒ™0

  BallastComm1TxData.index = 0;
	BallastComm1TxData.length = 0;
	memset(BallastComm1TxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff∏≥÷µŒ™0
}

static inline void BallastRxDataInput(BallastMessage *temp, char dat)
{
  if(temp->length < BALLAST_BUFF_SIZE)
  {
    temp->Buff[temp->index] = dat;
    temp->index++;
    temp->length++;
  }
}

static inline void BallastRxDataClear(BallastMessage *temp)
{
  memset(temp->Buff, 0, BALLAST_BUFF_SIZE);
	temp->index = 0;
	temp->length = 0;
}

void UART4_IRQHandler(void)
{
	unsigned char data;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
    data = USART_ReceiveData(UART4);
	  if(data == 0x02)
		{
			BallastRxDataClear(&BallastComm1RxData);
		}
			
		BallastRxDataInput(&BallastComm1RxData,data);
			
		if(data == 0x03)
		{
			BCC_CheckSum(BallastComm1RxData.Buff,BallastComm1RxData.length-3);
			xQueueSendFromISR(BallastComm1Queue, BallastComm1RxData.Buff, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
			BallastRxDataClear(&BallastComm1RxData);
		}
	}
}

void DMA1_Stream4_IRQHandler(void)//GSM_DMA∑¢ÀÕ÷–∂œ
{
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4); 
//	DMA_Cmd(DMA1_Stream4, DISABLE);
	xSemaphoreGive(ballastComm1Tx_semaphore);
}


void BallastComm1DMA_TxBuff(u8 *buf, u8 buf_size)
{
	if(xSemaphoreTake(ballastComm1Tx_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		memcpy(BallastComm1TxData.Buff, buf, buf_size);
		BallastComm1TxData.length = buf_size;
		
		DMA_Cmd(DMA1_Stream4, DISABLE);                                  //πÿ±’DMA¥´ ‰ 
		while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	             //»∑±£DMAø…“‘±ª…Ë÷√  
		DMA_SetCurrDataCounter(DMA1_Stream4,BallastComm1TxData.length);  // ˝æ›¥´ ‰¡ø  
		DMA_Cmd(DMA1_Stream4, ENABLE);                                   //ø™∆ÙDMA¥´ ‰ 
  }
}

static void BallastComm1InitHardware(void)
{
	BallastUartInit();
	Ballast_TX_DMA_Init();
}


static void vBallastComm1Task(void *parameter)
{
	u8 message[sizeof(BallastComm1RxData.Buff)];
	u8 protocol_type;
	
	for(;;)
	{
		if(xQueueReceive(BallastComm1Queue, &message, configTICK_RATE_HZ) == pdTRUE)
		{
			protocol_type = (chr2hex(message[5])<<4 | chr2hex(message[6]));
			const MessageHandlerMap *map = Ballast_MessageMaps;
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

void BallastCommInit(void)
{
	BallastComm1InitHardware();
	BallastComm1RxTxDataInit();
	vSemaphoreCreateBinary(ballastComm1Tx_semaphore);
	BallastComm1Queue = xQueueCreate(30, sizeof(BallastComm1RxData.Buff));
	xTaskCreate(vBallastComm1Task, "BallastComm1Task", ZIGBEE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 6, NULL);
}
