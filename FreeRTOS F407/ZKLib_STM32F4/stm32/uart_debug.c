#include <string.h>
#include <stdio.h>
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "misc.h"
#include "sys_debug.h"

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	

#define EN_USART_DEBUG   1

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{
	x = x; 
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
	USART6->DR = (u8) ch;    
  while((USART6->SR&0X40)==0);//ѭ������,ֱ���������     
	return ch;
}
#endif

#if EN_USART_DEBUG   //���ʹ�ܴ��ڵ���

void UartDebugHardwareInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*****************ʹ��IO�ںʹ���ʱ��*******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 
	/*****************���ڶ�Ӧ���Ÿ���ӳ��****************/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	
	/********************485��д*******************/
	RS485_RECEIVE_SEL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
	/********************USART�˿�����********************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   /*****************USART��ʼ������********************/
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(DEBUG_COM, &USART_InitStructure); //��ʼ������
	
  USART_Cmd(DEBUG_COM, ENABLE);  //ʹ�ܴ���
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(DEBUG_COM, USART_IT_RXNE, ENABLE);//��������ж�

	/*****************Usart1 NVIC����***********************/
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_COM_IRQn;//�����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ����
}

#endif






