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
	if(RTC_GetFlagStatus(RTC_FLAG_WUTF)==SET)//WK_UP�ж�?
  {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		RTC_ClearFlag(RTC_FLAG_WUTF);	//����жϱ�־
		xSemaphoreGiveFromISR(RTC_SystemRunningSemaphore, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken) 
		{
			taskYIELD();
		}
  }						
}

//cnt:�Զ���װ��ֵ.����0,�����ж�
void RTC_Set_WakeUp(u32 wksel,u16 cnt)
{
	RTC_WakeUpCmd(DISABLE);//�ر�WAKE UP
	
	RTC_WakeUpClockConfig(wksel);//����ʱ��ѡ��
	
	RTC_SetWakeUpCounter(cnt);//����WAKE UP�Զ���װ�ؼĴ���

	RTC_ClearITPendingBit(RTC_IT_WUT); //���RTC WAKE UP�ı�־
	 
	RTC_ITConfig(RTC_IT_WUT,ENABLE);//����WAKE UP ��ʱ���ж�
	RTC_WakeUpCmd(ENABLE);//����WAKE UP ��ʱ����
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
}


u8 RtcInit(void)
{
  RTC_InitTypeDef RTC_InitStructure;
	
	vSemaphoreCreateBinary(RTC_SystemRunningSemaphore);
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//ʹ��PWRʱ��
	PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ������� 
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0xA5A5)		//�Ƿ��һ������? 0xA5A5Ϊ��Ϊ�����Ƿ����ñ�׼
	{
		RCC_LSEConfig(RCC_LSE_ON);//LSE ����
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);	//���ָ����RCC��־λ�������,�ȴ����پ������
			
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//����RTCʱ��(RTCCLK),ѡ��LSE��ΪRTCʱ��    
		RCC_RTCCLKCmd(ENABLE);	//ʹ��RTCʱ�� 

    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;//RTC�첽��Ƶϵ��(1~0X7F)
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;//RTCͬ����Ƶϵ��(0~7FFF)
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;//RTC����Ϊ,24Сʱ��ʽ
    RTC_Init(&RTC_InitStructure);
	 
		RTC_WriteBackupRegister(RTC_BKP_DR0,0xA5A5);	//����Ѿ���ʼ������
	}
	
  RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);
	return true;
}
