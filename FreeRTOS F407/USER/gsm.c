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
#include "electric.h"
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
extern xQueueHandle ElectricQueue;
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

const static WGMessageHandlerMap GPRS_MessageMaps[] =  //二位数组的初始化
{
	{GATEPARAM,         HandleGatewayParam},     /*0x01; 网关参数下载*/           
	{LAMPPARAM,         HandleLampParam},        /*0x02; 灯参数下载*/  
	{LAMPSTRATEGY,      HandleLampStrategy},     /*0x03; 灯策略下载*/
	{LAMPDIMMING,       HandleLampDimmer},       /*0x04; 灯调光控制*/
	{LAMPONOFF,         HandleLampOnOff},        /*0x05; 灯开关控制*/
	{READLAMPDATA,      HandleReadBSNData},      /*0x06; 读镇流器数据*/
	{BRANCHCTRL,        HandleBranchOnOff},      /*0x07; 网关回路控制*/
	{DATAQUERY,         HandleGWDataQuery},      /*0x08; 网关数据查询*/
	{UNITRUNBACK,       HandleUnitRunBack},      /*0x0A; 单灯自主运行*/
	{TIMEADJUST,        HandleAdjustTime},       /*0x0B; 校时*/
	{VERSIONQUERY,      HandleGWVersQuery},      /*0x0C; 查网关软件版本号*/ 
  {ELECVERSION,       HandleElecVersQuery},    /*0x0E; 查电量板软件版本号*/	
	{GWADDRQUERY,       HandleGWAddrQuery},      /*0x11; 查网关地址*/
	{SETSERVERIPPORT,   HandleSetIPPort},        /*0x14; 设置目标服务器IP和端口*/
	{GATEUPGRADE,       HandleGWUpgrade},        /*0x15; 网关远程升级*/
	{GPRSQUALITY,       HandleSignalQuality},    /*0x17; gprs信号强度*/
	{TUNNELSTRATEGY,    HandleTunnelStrategy},   /*0x22; 隧道策略下载*/                    
	{RESTART,           HandleRestart},          /*0x3F; 设备复位*/
	{SENDELEC,          HandleSend},             /*0x88; 网关数据查询*/
	{SENDELECSOFT,      HandleSend},             /*0x8E; 查电量采集软件版本号*/
	{SENDELEFTPUP,      HandleSend},             /*0x9E; 电量采集模块远程升级*/
  {SENDELERESET,      HandleSend},             /*0x3F; 电量采集模块复位*/	
  {WGPROTOCOL_NULL,   NULL},                    /*保留*/  	
};
 
static void GSM_USART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************使能IO口和串口时钟*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	/*****************串口对应引脚复用映射****************/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
	/********************USART端口配置********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   /*****************USART初始化设置********************/
	USART_InitStructure.USART_BaudRate = 57600;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;//发送接收硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(GSM_COM, &USART_InitStructure); //初始化串口
	
  USART_Cmd(GSM_COM, ENABLE);  //使能串口
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(GSM_COM, USART_IT_RXNE, ENABLE);//开启相关中断

	/*****************Usart1 NVIC配置***********************/
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器� 
}

static void GSM_CtrlPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/****************GSM电源引脚*******************/
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
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
  DMA_DeInit(DMA1_Stream3);//DMA1数据流3
	
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&GsmTxData.Buff;//DMA 存储器0地址
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
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA Stream
	
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
		
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=15;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器�
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
	memset(GsmRxData.Buff, 0, GSM_BUFF_SIZE);//Buff赋值为0
	
	GsmTxData.index = 0;
	GsmTxData.length = 0;
	memset(GsmTxData.Buff, 0, GSM_BUFF_SIZE);//Buff赋值为0
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
//		printf_buff(&data, 1);
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
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志
		
		memcpy(GsmTxData.Buff, buf, buf_size);
		GsmTxData.length = buf_size;
			
		DMA_Cmd(DMA1_Stream3, DISABLE);                         //关闭DMA传输 
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	    //确保DMA可以被设置  
		DMA_SetCurrDataCounter(DMA1_Stream3,GsmTxData.length);  //数据传输量  
				
		xQueueReceive(GSM_AT_queue, &message, 0);               //开启DMA传输之前清空AT消息队列
				
		DMA_Cmd(DMA1_Stream3, ENABLE);                          //开启DMA传输 
		
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
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 5))//连接服务器
	{
		printf_str("\r\n无法连接服务器IP端口\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 10))
	{
		printf_str("\r\n无法连接服务器IP端口\r\n");
		return ERROR;
	}
	
	return SUCCESS;
}

ErrorStatus SendMsgToSim(char *cmd, char *ack, u32 waittime)//发送at指令到gsm模块
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

void SimSendData(u8 *buf,u8 buf_size)//发送gprs数据到服务器
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
#if (RUN_DEBUG_MODE == 1)
	printf_str("\r\nSend: ");
	printf_buff(buf, buf_size);
#endif
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
		printf_str("命令模式切换失败\r");
		return ERROR;
	}
	return SUCCESS;
}

ErrorStatus SwitchToData(void)
{
	vTaskDelay(500/portTICK_RATE_MS);
	
	if(!SendMsgToSim("ATO\r\n", "CONNECT", configTICK_RATE_HZ * 5))
	{
		printf_str("数据模式切换失败\r");
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
		
		if(!SendMsgToSim(NULL, "NORMAL POWER DOWN", configTICK_RATE_HZ * 5))//GSM关闭正常
		{
			continue;
		}
		
		vTaskDelay(configTICK_RATE_HZ);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ );
		GPIO_SetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		vTaskDelay(configTICK_RATE_HZ * 3 / 2);
		GPIO_ResetBits(GPIO_GPRS_PW_EN, PIN_GPRS_PW_EN);
		
		if(SendMsgToSim(NULL, "SMS Ready", configTICK_RATE_HZ * 15))//等待SMS准备完成
	  {
			break;
	  }
	}
}

ErrorStatus GsmStartConnect(void)
{
	char buf[50];
	
	GSM_ModuleStart();//GSM上电重启
	
	if(!SendMsgToSim("AT\r\n", "OK", configTICK_RATE_HZ))//测试串口通讯是否正常
	{
		printf_str("\r\nGSM串口通信接口异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+IPR=57600\r\n", "OK", configTICK_RATE_HZ))//设置通信波特率57600
	{
	  printf_str("\r\nGSM通信波特率设置异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("ATE0\r\n", "OK", configTICK_RATE_HZ))//回显模式关闭
	{
	  printf_str("\r\n回显模式关闭异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CIPMODE=1\r\n", "OK", configTICK_RATE_HZ)) //选择TCPIP模式为透明传输
	{
	  printf_str("\r\n透明传输配置异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CIPCCFG=5,2,1024,1\r\n", "\r\nOK\r\n", configTICK_RATE_HZ)) //配置传输参数   间隔200ms
	{
	  printf_str("\r\nGSM传输参数配置异常！\r\n");
		return ERROR;
	}
	
	
	if(!SendMsgToSim("AT+CLTS=1\r\n", "OK", configTICK_RATE_HZ)) //获取本地时间戳
	{
	  printf_str("\r\nGSM获取时间异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CPIN?\r\n", "+CPIN: READY", configTICK_RATE_HZ * 2))//查询SIM卡是否READY
	{
		printf_str("\r\nSIM卡READY异常！\r\n");
		return ERROR;
	}
	
	vTaskDelay(configTICK_RATE_HZ);	
	
	if(!SendMsgToSim("AT+CREG?\r\n", "+CREG: 0,1", configTICK_RATE_HZ))//查询GSM网络注册信息
	{
		printf_str("\r\n网络注册信息异常！\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim("AT+CGATT=0\r", "OK", configTICK_RATE_HZ * 2)) //gprs分离
	{
		printf("\r\nGPRS分离异常\r\n");
		return ERROR;
  }
	
	vTaskDelay(configTICK_RATE_HZ * 3);
	
	if(!SendMsgToSim("AT+CGATT=1\r\n", "OK", configTICK_RATE_HZ * 10))//启动TCP连接
	{
		printf_str("\r\nGPRS附着状态异常！\r\n");
		return ERROR;
	}
	
	NorFlashRead(NORFLASH_ADDR_BASE + NORFLASH_IP1_PORT1, (u16 *)&WG_ServerParameter, (sizeof(WG_ServerParameterType) + 1)/ 2);	
	
	sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", WG_ServerParameter.serverIP, WG_ServerParameter.serverPORT);
	
	if(!SendMsgToSim(buf, "OK", configTICK_RATE_HZ * 10))//连接服务器
	{
		printf_str("\r\nTCP连接异常\r\n");
		return ERROR;
	}
	
	if(!SendMsgToSim(NULL, "\r\nCONNECT\r\n", configTICK_RATE_HZ * 15))
	{
		printf_str("\r\n无法连接服务器IP端口\r\n");
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
	strncpy((char*)p, (char*)ManagementAddr+7, 3);

  SimSendData(buf,3);
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
	u8 protocol_type,i,prin_size;
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
		if(!SendMsgToSim("AT+CCLK?\r\n", "+CCLK: ", configTICK_RATE_HZ))//更新网络时间
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}
		if(!SendMsgToSim("AT+CSQ\r\n", "+CSQ:", configTICK_RATE_HZ*2))//查询信号强度
		{
			if(GsmStartConnect() == ERROR)
			  NVIC_SystemReset();
		}
		if(uxQueueMessagesWaiting(GSM_GPRS_queue) <= 1)//发送网关数据
		{
			WG_DataSample(0xFF);
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
				for(i=0;i<3;i++)//一次最多传输3个单灯数据86数据
				{
					if(xQueueReceive(GPRSSendAddrQueue, &addr[i], 1000/portTICK_RATE_MS) == pdFALSE)//最大等待一秒
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
#if (RUN_DEBUG_MODE == 1)
					printf_str("\r\nReceive: ");
					prin_size = (chr2hex(*(message+13))<<4 | chr2hex(*(message+14)));
					printf_buff(message, prin_size+18);
#endif			
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
					if(!SendMsgToSim("AT+CIPSTATUS\r\n", "STATE: CONNECT OK", configTICK_RATE_HZ))//检测TCP连接状况
					{
						if(GsmStartConnect() == ERROR)
							NVIC_SystemReset();
					}
					if(!SendMsgToSim("AT+CSQ\r\n", "+CSQ:", configTICK_RATE_HZ*2))//查询信号强度
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
	xTaskCreate(vGSMTask, "GSMTask", GSM_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &xGSMTaskHandle);
	/*************************************************************************************************
	 由于GSM模块包含透传和非透传模式，而且在工作过程中需要切换，故GPRS发送接收必须在一个任务内
	 *************************************************************************************************/
}

/*******************************END OF FILE************************************/
