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
#include "electric.h"
#include "table_process.h"


#define ZIGBEE_TASK_STACK_SIZE		     (configMINIMAL_STACK_SIZE + 1024*2)
#define BALLAST_BUFF_SIZE   100

#define  BALLAST_COMM1      UART4

xSemaphoreHandle ballastComm1Tx_semaphore;
static xQueueHandle  BallastComm1Queue;
extern LampAttrSortType LampAttrSortTable[MAX_LAMP_NUM];
extern LampRunCtrlType LampRunCtrlTable[MAX_LAMP_NUM];
extern LampBuffType LampAddr;
extern xQueueHandle  LampQueryAddrQueue;
extern xQueueHandle  GPRSSendAddrQueue;
u8 UnitWaitFlag;
u8 UnitQueryState;
extern u16 QueryUnitAddrBCD;
extern u8 WG_State;
extern AlarmParmTypeDef AlarmParm;
extern WG_AlarmFlagDef WG_AlarmFlag;

typedef struct
{
	u8 index;
	u8 length;
  u8 Buff[BALLAST_BUFF_SIZE];
}BallastMessage;


const static UnitMessageHandlerMap Ballast_MessageMaps[] =  //¶þÎ»Êý×éµÄ³õÊ¼»¯
{
	{UNITREADDATABACK,     HandleUnitReadDataReply},       /*0x86; ¶ÁÕòÁ÷Æ÷Êý¾Ý*/  
  {UNITPARAMBACK,        HandleUnitLightParamReply},     /*0x82; µÆ²ÎÊýÏÂÔØ*/   	
	{UNITSTRATEGYBACK,     HandleUnitStrategyReply},       /*0x83; µÆ²ßÂÔÏÂÔØ*/
  {UNITPROTOCOL_NULL,    NULL},                          /*±£Áô*/  
};

BallastMessage BallastComm1RxData;
BallastMessage BallastComm1TxData;

static void BallastUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************Ê¹ÄÜIO¿ÚºÍ´®¿ÚÊ±ÖÓ*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
 
	/*****************´®¿Ú¶ÔÓ¦Òý½Å¸´ÓÃÓ³Éä****************/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	/********************USART¶Ë¿ÚÅäÖÃ********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   /*****************USART³õÊ¼»¯ÉèÖÃ********************/
	USART_InitStructure.USART_BaudRate = 38400;//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½
  USART_Init(UART4, &USART_InitStructure); //³õÊ¼»¯´®¿Ú
	
  USART_Cmd(UART4, ENABLE);  //Ê¹ÄÜ´®¿Ú
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//¿ªÆôÏà¹ØÖÐ¶Ï

	/*****************Usart1 NVICÅäÖÃ***********************/
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//´®¿ÚÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//ÇÀÕ¼ÓÅÏÈ¼¶15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//×ÓÓÅÏÈ¼¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡
}

static void Ballast_TX_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1Ê±ÖÓÊ¹ÄÜ 
  DMA_DeInit(DMA1_Stream4);//DMA1Êý¾ÝÁ÷4
	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//µÈ´ýDMA¿ÉÅäÖÃ 
	
  /* ÅäÖÃ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Í¨µÀÑ¡Ôñ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMAÍâÉèµØÖ·
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&BallastComm1TxData.Buff;//DMA ´æ´¢Æ÷0µØÖ·
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
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//³õÊ¼»¯DMA Stream
		
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //Ê¹ÄÜ´®¿Ú4µÄDMA·¢ËÍ
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//´®¿ÚÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//×ÓÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡
}

static void BallastComm1RxTxDataInit(void)
{
  BallastComm1RxData.index = 0;
	BallastComm1RxData.length = 0;
	memset(BallastComm1RxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff¸³ÖµÎª0

  BallastComm1TxData.index = 0;
	BallastComm1TxData.length = 0;
	memset(BallastComm1TxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff¸³ÖµÎª0
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

void DMA1_Stream4_IRQHandler(void)//GSM_DMA·¢ËÍÖÐ¶Ï
{
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4); 
//	DMA_Cmd(DMA1_Stream4, DISABLE);
	xSemaphoreGive(ballastComm1Tx_semaphore);
}


void UnitComm1DMA_TxBuff(u8 *buf, u8 buf_size)
{
	if(xSemaphoreTake(ballastComm1Tx_semaphore, configTICK_RATE_HZ * 5) == pdTRUE) 
  {
		memcpy(BallastComm1TxData.Buff, buf, buf_size);
		BallastComm1TxData.length = buf_size;
		
		DMA_Cmd(DMA1_Stream4, DISABLE);                                  //¹Ø±ÕDMA´«Êä 
		while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	             //È·±£DMA¿ÉÒÔ±»ÉèÖÃ  
		DMA_SetCurrDataCounter(DMA1_Stream4,BallastComm1TxData.length);  //Êý¾Ý´«ÊäÁ¿  
		DMA_Cmd(DMA1_Stream4, ENABLE);                                   //¿ªÆôDMA´«Êä 
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
	u8 protocol_type,i;
	u8 wait_count = 0;
	u16 QueueAddrBCD,recv_addr_bcd,unit_addr_hex;
	
  vTaskDelay(3000/portTICK_RATE_MS);//ÑÓ³Ù3s²ÉÑùµ¥µÆÊý¾Ý
	
	UnitWaitFlag = WAIT_READDATA_REPLY;
	UnitQueryState = AUTO_QUERY;
	
	for(;;)
	{
		while(1)//±»¶¯ÂÖÑ¯
		{
			UnitQueryState = PASSIVE_QUERY;
			
			if(uxQueueMessagesWaiting(LampQueryAddrQueue) == 0)
        break;
			else
			{
				if(xQueueReceive(LampQueryAddrQueue, &QueueAddrBCD, 0) == pdTRUE)//Íø¹ØÂÖÑ¯Ö¸¶¨µØÖ·µ¥µÆÆ
				{
					ReadUnitData(QueueAddrBCD);
					unit_addr_hex = Bcd2ToByte(QueryUnitAddrBCD>>8)*100 + Bcd2ToByte(QueryUnitAddrBCD & 0x00FF);
				}
				for(i=1;i<=MAX_QUERY_NUM;i++)
				{
					if(xQueueReceive(BallastComm1Queue, &message, 1000/portTICK_RATE_MS) == pdTRUE)//µÈ´ý1s
					{
						recv_addr_bcd =  chr2hex(message[1])<<4;
						recv_addr_bcd = (chr2hex(message[2])+recv_addr_bcd)<<4;
						recv_addr_bcd = (chr2hex(message[3])+recv_addr_bcd)<<4;
						recv_addr_bcd =  chr2hex(message[4])+recv_addr_bcd;
						
					  if(recv_addr_bcd == QueryUnitAddrBCD)
						{
						  protocol_type = (chr2hex(message[5])<<4 | chr2hex(message[6]));
							if(protocol_type == 0x86)
							{
								HandleUnitReadDataReply(message);
								break;
							}
					  }
					}
					else
					{
						if(i == LampRunCtrlTable[unit_addr_hex].query_num)
						{
							if(LampRunCtrlTable[unit_addr_hex].run_state != HARDWARE_CLOSE)
							{
								clear_unit_buff(CONNECT_FAIL, QueueAddrBCD);
								break;
							}
						}
						else
						  ReadUnitData(QueueAddrBCD);
					}
				}
		  }
		}
		
		ReadUnitData(LampAttrSortTable[LampAddr.index].addr);//Ìø×ªÊ±²É¼¯Ò»´Îµ¥µÆÊý¾Ý
		
		while(1)//Ö÷¶¯ÂÖÑ¯
		{
			UnitQueryState = AUTO_QUERY;
			
			if(uxQueueMessagesWaiting(LampQueryAddrQueue) != 0)
			{
				vTaskDelay(3000/portTICK_RATE_MS);//ÑÓ³Ù3s±»¶¯²ÉÑùµ¥µÆÊý¾Ý
			  break;
			}
      else
      {
				if(xQueueReceive(BallastComm1Queue, &message, 100/portTICK_RATE_MS) == pdTRUE)//µÈ´ý100ms
				{
					recv_addr_bcd =  chr2hex(message[1])<<4;
					recv_addr_bcd = (chr2hex(message[2])+recv_addr_bcd)<<4;
					recv_addr_bcd = (chr2hex(message[3])+recv_addr_bcd)<<4;
					recv_addr_bcd =  chr2hex(message[4])+recv_addr_bcd;
					
					if(recv_addr_bcd == QueryUnitAddrBCD)
					{
						protocol_type = (chr2hex(message[5])<<4 | chr2hex(message[6]));
						const UnitMessageHandlerMap *map = Ballast_MessageMaps;
						for(; map->type != UNITPROTOCOL_NULL; ++map)
						{
							if(protocol_type == map->type) 
							{
								map->handlerFunc(message);
								break;
							}
						}
						
						if(UnitWaitFlag == WAIT_READDATA_REPLY)
						  QueryNextAddr();
						wait_count = 0;
				  }
			  }
				else
				{
					wait_count++;
					
					unit_addr_hex = Bcd2ToByte(QueryUnitAddrBCD>>8)*100 + Bcd2ToByte(QueryUnitAddrBCD & 0x00FF);
					
					if((UnitWaitFlag == WAIT_READDATA_REPLY) && (wait_count < LampRunCtrlTable[unit_addr_hex].query_num*10))
					{
						if(wait_count%10 == 0)
						{
						  ReadUnitData(QueryUnitAddrBCD);
					  }
					}
					else if((UnitWaitFlag == WAIT_PARAM_REPLY) && (wait_count <= 20))
					{
					}
					else if((UnitWaitFlag == WAIT_STRATEGY_REPLY) && (wait_count <= 40))
					{
					}
					else
					{
						if(UnitWaitFlag == WAIT_READDATA_REPLY)
						{
							if(QueryUnitAddrBCD != 0)
						    clear_unit_buff(CONNECT_FAIL, QueryUnitAddrBCD);//Í¨ÐÅÊ§°ÜÊý¾Ý¸üÐÂ
							
							if(LampRunCtrlTable[unit_addr_hex].query_num > 1)
							  LampRunCtrlTable[unit_addr_hex].query_num--;
						}
						wait_count = 0;
						QueryNextAddr();
					}
				}
			}
		}
		xQueueReceive(BallastComm1Queue, &message, 1000/portTICK_RATE_MS);//Çå¿ÕÒ»´ÎÖ÷¶¯ÂÖÑ¯·µ»ØµÄµ¥µÆÊý¾Ý
	}
}

TaskHandle_t xBallastComm1Task;

void BallastCommInit(void)
{
	BallastComm1InitHardware();
	BallastComm1RxTxDataInit();
	vSemaphoreCreateBinary(ballastComm1Tx_semaphore);
	BallastComm1Queue = xQueueCreate(30, sizeof(BallastComm1RxData.Buff));
	xTaskCreate(vBallastComm1Task, "BallastComm1Task", ZIGBEE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xBallastComm1Task);
}
