#ifndef __SYS_DEBUG_H
#define __SYS_DEBUG_H

#define DEBUG_COM       USART6
#define DEBUG_COM_IRQn  USART6_IRQn

#define RS485_SEND_SEL      GPIO_SetBits(GPIOC, GPIO_Pin_8);    
#define RS485_RECEIVE_SEL   GPIO_ResetBits(GPIOC, GPIO_Pin_8);


void UartDebugInit(void);
void UartDebugHardwareInit(void);
void printf_str(char *str);
void printf_buff(u8 *buf, u16 buf_size);

#endif
