#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

u8 BCC_CheckSum(u8 *buf, u8 len);
u8 chr2hex(u8 chr);
u8 hex2chr(u8 hex);

#endif
