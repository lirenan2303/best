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
extern u8 UnitErrorNum;
extern AlarmParmTypeDef AlarmParm;
extern WG_AlarmFlagDef WG_AlarmFlag;

typedef struct
{
	u8 index;
	u8 length;
  u8 Buff[BALLAST_BUFF_SIZE];
}BallastMessage;


const static UnitMessageHandlerMap Ballast_MessageMaps[] =  //二位数组的初始化
{
	{UNITREADDATABACK,     HandleUnitReadDataReply},       /*0x86; 读镇流器数据*/  
  {UNITPARAMBACK,        HandleUnitLightParamReply},     /*0x82; 灯参数下载*/   	
	{UNITSTRATEGYBACK,     HandleUnitStrategyReply},       /*0x83; 灯策略下载*/
  {UNITPROTOCOL_NULL,    NULL},                          /*保留*/  
};

BallastMessage BallastComm1RxData;
BallastMessage BallastComm1TxData;

static void BallastUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************使能IO口和串口时钟*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
 
	/*****************串口对应引脚复用映射****************/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	/********************USART端口配置********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   /*****************USART初始化设置********************/
	USART_InitStructure.USART_BaudRate = 38400;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口
	
  USART_Cmd(UART4, ENABLE);  //使能串口
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	/*****************Usart1 NVIC配置***********************/
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�
}

static void Ballast_TX_DMA_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
  DMA_DeInit(DMA1_Stream4);//DMA1数据流4
	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&BallastComm1TxData.Buff;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0x00;//数据传输量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //指定使用FIFO模式还是直接模式        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//制定了FIFO阈值
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//初始化DMA Stream
		
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //使能串口4的DMA发送
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�
}

static void BallastComm1RxTxDataInit(void)
{
  BallastComm1RxData.index = 0;
	BallastComm1RxData.length = 0;
	memset(BallastComm1RxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff赋值为0

  BallastComm1TxData.index = 0;
	BallastComm1TxData.length = 0;
	memset(BallastComm1TxData.Buff, 0, BALLAST_BUFF_SIZE);//Buff赋值为0
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

void DMA1_Stream4_IRQHandler(void)//GSM_DMA发送中断
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
		
		DMA_Cmd(DMA1_Stream4, DISABLE);                                  //关闭DMA传输 
		while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	             //确保DMA可以被设置  
		DMA_SetCurrDataCounter(DMA1_Stream4,BallastComm1TxData.length);  //数据传输量  
		DMA_Cmd(DMA1_Stream4, ENABLE);                                   //开启DMA传输 
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
	
  vTaskDelay(2000/portTICK_RATE_MS);//延迟2s采样单灯数据
	
	UnitWaitFlag = WAIT_READDATA_REPLY;
	UnitQueryState = AUTO_QUERY;
	
	for(;;)
	{
		while(1)//被动轮询
		{
			UnitQueryState = PASSIVE_QUERY;
			
			if(uxQueueMessagesWaiting(LampQueryAddrQueue) == 0)
        break;
			else
			{
				if(xQueueReceive(LampQueryAddrQueue, &QueueAddrBCD, 0) == pdTRUE)//网关轮询指定地址单灯�
				{
					ReadUnitData(QueueAddrBCD);
					unit_addr_hex = Bcd2ToByte(QueryUnitAddrBCD>>8)*100 + Bcd2ToByte(QueryUnitAddrBCD & 0x00FF);
				}
				for(i=1;i<=MAX_QUERY_NUM;i++)
				{
					if(xQueueReceive(BallastComm1Queue, &message, 1000/portTICK_RATE_MS) == pdTRUE)//等待1s
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
		
		ReadUnitData(LampAttrSortTable[LampAddr.index].addr);//跳转时采集一次单灯数据
		
		while(1)//主动轮询
		{
			UnitQueryState = AUTO_QUERY;
			
			if(uxQueueMessagesWaiting(LampQueryAddrQueue) != 0)
			{
				vTaskDelay(1200/portTICK_RATE_MS);//延迟1.2s被动采样单灯数据
			  break;
			}
      else
      {
				if(xQueueReceive(BallastComm1Queue, &message, 100/portTICK_RATE_MS) == pdTRUE)//等待100ms
				{
					recv_addr_bcd =  chr2hex(message[1])<<4;
					recv_addr_bcd = (chr2hex(message[2])+recv_addr_bcd)<<4;
					recv_addr_bcd = (chr2hex(message[3])+recv_addr_bcd)<<4;
					recv_addr_bcd =  chr2hex(message[4])+recv_addr_bcd;
					
					if(recv_addr_bcd == QueryUnitAddrBCD)
					{
						UnitErrorNum = 0;//清除连续灯不亮数目
						
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
						    clear_unit_buff(CONNECT_FAIL, QueryUnitAddrBCD);//通信失败数据更新
							
							if(LampRunCtrlTable[unit_addr_hex].query_num > 1)
							  LampRunCtrlTable[unit_addr_hex].query_num--;
						}
						wait_count = 0;
						QueryNextAddr();
					}
				}
			}
		}
		xQueueReceive(BallastComm1Queue, &message, 1000/portTICK_RATE_MS);//清空一次主动轮询返回的单灯数据
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
