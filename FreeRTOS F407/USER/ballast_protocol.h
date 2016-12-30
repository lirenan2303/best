#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

#include "stm32f4xx.h"


typedef enum
{
	UNITPARAMBACK    = 0x82,        /*灯参数下载*/
	UNITSTRATEGYBACK = 0x83,        /*灯策略下载*/
	UNITREADDATABACK = 0x86,        /*读镇流器数据*/
 	UNITPROTOCOL_NULL = 0x00,       /*保留*/
}WGUnitProtocolRxType;

typedef enum
{
	UNITPARAM            = 0x02,   
	UNITSTRATEGY         = 0x03,
	UNITDIMMING          = 0x04,
	UNITONOFF            = 0x05,
	UNITDATA             = 0x06,
	UNITCTRLBACK         = 0x0A,  
	UNITTIMEUPATE        = 0x0B,
}WGUnitProtocolTxType;

typedef enum
{
	WAIT_PARAM_REPLY    = 0x02,   /*灯参数下载应答*/
	WAIT_STRATEGY_REPLY = 0x03,   /*灯策略下载应答*/
	WAIT_READDATA_REPLY = 0x06,   /*读镇流器数据应答*/
	UNIT_QUERY_START = 0x00,
}WAITUNITREPLY;

typedef enum {AUTO_QUERY = 0, PASSIVE_QUERY= !AUTO_QUERY} QuerySchemeStatus;

typedef struct
{
	u16 addr_bcd;
	u8  dimming;
	u8  state;
	u16 vin;
	u16 current;
	u16 power;
}LampDataTypeDef;

typedef struct
{
	WGUnitProtocolRxType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
}UnitMessageHandlerMap;

void ReadUnitData(u16 addr_bcd);
void QueryNextAddr(void);
void CtrlUnitSend(u8 *p, u8 size);
void HandleUnitLightParamReply(u8 *p);
void HandleUnitStrategyReply(u8 *p);
void HandleUnitReadDataReply(u8 *p);
void clear_unit_buff(u8 state, u16 unit_addr_bcd);


#endif
