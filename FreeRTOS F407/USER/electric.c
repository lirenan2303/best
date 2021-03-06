#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <string.h>
#include <stdlib.h>
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
#include "km_ctrl.h"

#define ELECTRIC_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 1024*1)
#define ELECTRIC_BUFF_SIZE  200 //与gprs队列必须相等

SemaphoreHandle_t    EleTx_semaphore;
xQueueHandle ElectricQueue;
extern LampBuffType LampAddr;
extern AlarmParmTypeDef AlarmParm;
extern xQueueHandle  GSM_GPRS_queue;
WG_AlarmFlagDef WG_AlarmFlag;

typedef struct 
{
	u8 index;
	u8 length;
  u8 Buff[ELECTRIC_BUFF_SIZE];
}ElectricMessage;

const static ElecMessageHandlerMap Electric_MessageMaps[] =  //二位数组的初始化
{
	{ELECQUERYACK,      ElecHandleGWDataQuery},       /*0x08; 网关数据查询*/
	{ELECSOFTQUERYACK,  ElecHandleSoftVerQuery},      /*0x0E; 查电量采集软件版本号*/
	{ELEFTPUPDATAACK,   ElecHandleFTPUpdata},         /*0x1E; 电量采集模块远程升级*/
  {ELERESETACK,       ElecHandleReset},             /*0x3F; 电量采集模块复位*/	
};


ElectricMessage EleTxData;
ElectricMessage EleRxData;

static void ElectricUartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************使能IO口和串口时钟*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
	/*****************串口对应引脚复用映射****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	/********************USART端口配置********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART初始化设置********************/
	USART_InitStructure.USART_BaudRate = 9600;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口
	
  USART_Cmd(USART1, ENABLE);  //使能串口
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	/*****************Usart1 NVIC配置***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�	
}

static void Electric_TX_DMA_Init(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA1时钟使能 
  DMA_DeInit(DMA2_Stream7);//DMA1数据流4
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&EleTxData.Buff;//DMA 存储器0地址
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
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//初始化DMA Stream
		
  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�
}

static void EleRxTxDataInit(void)
{
  EleRxData.index = 0;
	EleRxData.length = 0;
	memset(EleRxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff赋值为0

  EleTxData.index = 0;
	EleTxData.length = 0;
	memset(EleTxData.Buff, 0, ELECTRIC_BUFF_SIZE);//Buff赋值为0
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
//		printf_buff(&data, 1);//调试
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

void DMA2_Stream7_IRQHandler(void)//GSM_DMA发送中断
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
		
		DMA_Cmd(DMA2_Stream7, DISABLE);                         //关闭DMA传输 
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	    //确保DMA可以被设置  
		DMA_SetCurrDataCounter(DMA2_Stream7,EleTxData.length);  //数据传输量  
		DMA_Cmd(DMA2_Stream7, ENABLE);                          //开启DMA传输 
  }
}

static void ElectrolHardwareInit(void)
{
	ElectricUartInit();
	Electric_TX_DMA_Init();
}

void WG_StateClear(void)
{
	WG_AlarmFlag.cable_fault = 0;
	WG_AlarmFlag.water_alarm = 0;
	WG_AlarmFlag.door_open = 0;
	WG_AlarmFlag.temp_fault = 0;
	
	WG_AlarmFlag.light_fault = 0;
  WG_AlarmFlag.cont_off = 0;
	WG_AlarmFlag.work_cur_high = 0;
	WG_AlarmFlag.noload_cur_high = 0;
	
	WG_AlarmFlag.vol_low = 0;
	WG_AlarmFlag.vol_high = 0;
	WG_AlarmFlag.power_down = 0;
	WG_AlarmFlag.end_loss = 0;

	WG_AlarmFlag.lose_phase = 0;
	WG_AlarmFlag.pole_fault = 0;
}

void WG_AlarmFlagCheck(u8 *buf)
{
	WG_EleTypeDef WG_Ele;
	u8 temp[5]={0};
	
	strncpy((char *)temp, (char *)buf, 4);
	WG_Ele.L1_Vol = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)buf+4, 4);
	WG_Ele.L2_Vol = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)buf+8, 4);
	WG_Ele.L3_Vol = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)+12, 4);
	WG_Ele.L1_Cur = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)+16, 4);
	WG_Ele.L2_Cur = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)+20, 4);
	WG_Ele.L3_Cur = atoi((const char *)temp);
	
	strncpy((char *)temp, (char *)+24, 4);
	WG_Ele.Zero_Cur = atoi((const char *)temp);
	
	if((WG_Ele.L1_Vol <= 5) && (WG_Ele.L2_Vol <= 5) && (WG_Ele.L3_Vol <= 5))//掉电
	  WG_AlarmFlag.power_down = 1;
	else if((WG_Ele.L1_Vol <= 5) || (WG_Ele.L2_Vol <= 5) || (WG_Ele.L3_Vol <= 5))//缺相
	  WG_AlarmFlag.power_down = 1;
	else if((WG_Ele.L1_Vol < AlarmParm.L1_VolLow) || (WG_Ele.L2_Vol < AlarmParm.L2_VolLow) || (WG_Ele.L3_Vol < AlarmParm.L3_VolLow))//电压过低
	  WG_AlarmFlag.vol_low = 1;
	
	if((WG_Ele.L1_Vol > AlarmParm.L1_VolHigh) || (WG_Ele.L2_Vol > AlarmParm.L2_VolHigh) || (WG_Ele.L3_Vol > AlarmParm.L3_VolHigh))//电压过高
	  WG_AlarmFlag.vol_high = 1;

  if((WG_Ele.L1_Cur > AlarmParm.L1_CurHigh) || (WG_Ele.L2_Cur > AlarmParm.L2_CurHigh) || (WG_Ele.L3_Cur > AlarmParm.L3_CurHigh) || (WG_Ele.Zero_Cur > AlarmParm.Zero_CurHigh))//电流过高
	  WG_AlarmFlag.work_cur_high = 1;
}

void WG_DataSample(u8 branch)
{
	u8 checksum;
  u8 buf_1[30]= {0x02,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x30,0x38,0x30,0x31};
	u8 buf_2[]= {0x02,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x39,0x30,0x38,0x30,0x39,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x33,0x42,0x03};
  
	if(branch == 0xFF)
	{
		xQueueSend(GSM_GPRS_queue, &buf_2, configTICK_RATE_HZ*5);//网关数据查询
	}
	else if(branch <= 8)
	{
		buf_1[15] = hex2chr(branch);
		
		checksum = BCC_CheckSum(buf_1, 16);
		buf_1[16] = hex2chr((checksum>>4)&0x0F);
		buf_1[17] = hex2chr( checksum & 0x0F  );
		
		buf_1[18] = 0x03;
		
		xQueueSend(GSM_GPRS_queue, &buf_1, configTICK_RATE_HZ*5);//网关数据查询
	}
}

u16 WG_StateCheck(void)
{
	u16 state=0;
	
	if(WG_AlarmFlag.cable_fault == 1)
		state |= 0x0001; 
	if(WG_AlarmFlag.water_alarm == 1)
		state |= 0x0002;
	if(WG_AlarmFlag.door_open == 1)
		state |= 0x0004;
	if(WG_AlarmFlag.temp_fault == 1)
	  state |= 0x0008;
	if(WG_AlarmFlag.light_fault == 1)
		state |= 0x0010;
  if(WG_AlarmFlag.cont_off == 1)
	  state |= 0x0020;
	if(WG_AlarmFlag.work_cur_high == 1)
		state |= 0x0040;
	if(WG_AlarmFlag.noload_cur_high == 1)
		state |= 0x0080;
	if(WG_AlarmFlag.vol_low == 1)
		state |= 0x0100;
	if(WG_AlarmFlag.vol_high == 1)
		state |= 0x0200;
	if(WG_AlarmFlag.power_down == 1)
		state |= 0x0400;
	if(WG_AlarmFlag.end_loss == 1)
    state |= 0x0800;
	if(WG_AlarmFlag.lose_phase == 1)
		state |= 0x1000;
	if(WG_AlarmFlag.pole_fault == 1)
		state |= 0x2000;
	
	if((state&0x3FFF) != 0)
		state |= 0x8000;
	
	return state;
}

void ElecHandleGWDataQuery(u8 *p) 
{
	u8 data_size=0;
	u8 *Alar_buf;
	u16 WriteBuff[20];
	u16 lamp_num_bcd;
	u16 WG_State,KM_State;
	
	Alar_buf = p+16;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
  WG_AlarmFlagCheck(Alar_buf);
	WG_State = WG_StateCheck();
	*(p+16) = hex2chr((WG_State>>12) & 0x000F);
	*(p+17) = hex2chr((WG_State>>8) & 0x000F);
	*(p+18) = hex2chr((WG_State>>4) & 0x000F);
	*(p+19) = hex2chr((WG_State) & 0x000F);
	
	KM_State = KM_Unit_branch_state();
	*(p+20) = hex2chr((KM_State>>4) & 0x000F);
	*(p+21) = hex2chr((KM_State) & 0x000F);
	
	NorFlashRead(NORFLASH_MANAGER_PARA1_BASE + NORFLASH_MANAGER_ID_OFFSET, WriteBuff, 6); 
  ConvertToByte(WriteBuff, 6, p+134);
	
	lamp_num_bcd = ByteToBcd2(LampAddr.num>>8)*256 + ByteToBcd2((u8)LampAddr.num);
	
	*(p+140) = hex2chr((lamp_num_bcd>>12) & 0x000F);
	*(p+141) = hex2chr((lamp_num_bcd>>8) & 0x000F);
	*(p+142) = hex2chr((lamp_num_bcd>>4) & 0x000F);
	*(p+143) = hex2chr((lamp_num_bcd) & 0x000F);
	
	RTC_TimeToChar(WriteBuff);
  ConvertToByte(WriteBuff, 12, p+144);
	
//	GPRS_Protocol_Response(ELECQUERYACK, p+15, data_size);
  xQueueSend(GSM_GPRS_queue, p, configTICK_RATE_HZ*5);
}

void ElecHandleSoftVerQuery(u8 *p)
{
	u8 data_size;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
//	GPRS_Protocol_Response(ELECSOFTQUERYACK, p+15, 3);
	xQueueSend(GSM_GPRS_queue, p, configTICK_RATE_HZ*5);
}

void ElecHandleFTPUpdata(u8 *p)
{
	
}

void ElecHandleReset(u8 *p)
{
	u8 data_size;
	
	if(!GatewayAddrCheck(p+1))
		return;
	
	if(!GPRS_Protocol_Check(p, &data_size))
    return;
	
//	GPRS_Protocol_Response(ELERESETACK, p+15, 1);
	xQueueSend(GSM_GPRS_queue, p, configTICK_RATE_HZ*5);
}

static void vElectTask(void *parameter)
{
	u8 message[sizeof(EleRxData.Buff)];
	u8 protocol_type;
	
	for(;;)
	{
		if(xQueueReceive(ElectricQueue, &message, portMAX_DELAY) == pdTRUE)
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
	WG_StateClear();
	xTaskCreate(vElectTask, "ElectTask", ELECTRIC_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &xElectTask);
}
