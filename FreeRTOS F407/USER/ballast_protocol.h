#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

typedef enum
{
	UNITLIGHTPARAM = 0x82,      /*灯参数下载*/
	UNITSTRATEGY = 0x83,        /*灯策略下载*/
//	DIMMING = 0x04,         /*灯调光控制*/
//	LAMPSWITCH = 0x05,      /*灯开关控制*/
	UNITREADDATA = 0x86,        /*读镇流器数据*/
//	UNITUPDATA = 0X2A,  /*镇流器远程升级*/
//	TIMEADJUST = 0x42,      /*校时*/
 	UNITPROTOCOL_NULL,          /*保留*/
}UnitProtocolType;

typedef struct
{
	UnitProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
} UnitMessageHandlerMap;

void HandleUnitLightParam(u8 *p);
void HandleUnitStrategy(u8 *p);
void HandleUnitReadData(u8 *p);


#endif
