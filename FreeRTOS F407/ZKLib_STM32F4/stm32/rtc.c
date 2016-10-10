#include "stdio.h"
#include "rtc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "delay.h"

static xSemaphoreHandle RTC_SystemRunningSemaphore;
NVIC_InitTypeDef   NVIC_InitStructure;


void RTC_WKUP_IRQHandler(void)
{
	if(RTC_GetFlagStatus(RTC_FLAG_WUTF)==SET)//WK_UP中断?
  {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		RTC_ClearFlag(RTC_FLAG_WUTF);	//清除中断标志
		xSemaphoreGiveFromISR(RTC_SystemRunningSemaphore, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken) 
		{
			taskYIELD();
		}
  }						
}

//cnt:自动重装载值.减到0,产生中断
void RTC_Set_WakeUp(u32 wksel,u16 cnt)
{
	RTC_WakeUpCmd(DISABLE);//关闭WAKE UP
	
	RTC_WakeUpClockConfig(wksel);//唤醒时钟选择
	
	RTC_SetWakeUpCounter(cnt);//设置WAKE UP自动重装载寄存器

	RTC_ClearITPendingBit(RTC_IT_WUT); //清除RTC WAKE UP的标志
	 
	RTC_ITConfig(RTC_IT_WUT,ENABLE);//开启WAKE UP 定时器中断
	RTC_WakeUpCmd(ENABLE);//开启WAKE UP 定时器　
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
}


u8 RtcInit(void)
{
  RTC_InitTypeDef RTC_InitStructure;
	
	vSemaphoreCreateBinary(RTC_SystemRunningSemaphore);
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
	PWR_BackupAccessCmd(ENABLE);	//使能后备寄存器访问 
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0xA5A5)		//是否第一次配置? 0xA5A5为人为定义是否配置标准
	{
		RCC_LSEConfig(RCC_LSE_ON);//LSE 开启
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);	//检查指定的RCC标志位设置与否,等待低速晶振就绪
			
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//设置RTC时钟(RTCCLK),选择LSE作为RTC时钟    
		RCC_RTCCLKCmd(ENABLE);	//使能RTC时钟 

    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;//RTC异步分频系数(1~0X7F)
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;//RTC同步分频系数(0~7FFF)
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;//RTC设置为,24小时格式
    RTC_Init(&RTC_InitStructure);
	 
		RTC_WriteBackupRegister(RTC_BKP_DR0,0xA5A5);	//标记已经初始化过了
	}
	
  RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);
	return true;
}
