#include <string.h>
#include <stdio.h>
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "misc.h"
#include "sys_debug.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	

#define EN_USART_DEBUG   1

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{
	x = x; 
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
	USART6->DR = (u8) ch;    
  while((USART6->SR&0X40)==0);//循环发送,直到发送完毕     
	return ch;
}
#endif

#if EN_USART_DEBUG   //如果使能串口调试

void UartDebugHardwareInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************使能IO口和串口时钟*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 
	/*****************串口对应引脚复用映射****************/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	
	/********************485读写*******************/
	RS485_RECEIVE_SEL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	/********************USART端口配置********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   /*****************USART初始化设置********************/
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(DEBUG_COM, &USART_InitStructure); //初始化串口
	
  USART_Cmd(DEBUG_COM, ENABLE);  //使能串口
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(DEBUG_COM, USART_IT_RXNE, ENABLE);//开启相关中断

	/*****************Usart1 NVIC配置***********************/
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_COM_IRQn;//串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器�
}

#endif






