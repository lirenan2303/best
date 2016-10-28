#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

#define MAX_LAMP_NUM 1000
#define MAX_BRANCH_NUM 8
#define LAMP_DATA_TEMP_SIZE 50

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


typedef struct{
u8 branch;
u8 segment;
u16 pole_id; 
u16 addr;
}LampAttrSortType;

typedef struct{
u16 addr;
u8 query_state;
u8 run_state;
u8 dimming_value;
u8 retain;
}LampRunCtrlType;

void AllTableInit(void);
void HandleUnitLightParam(u8 *p);
void HandleUnitStrategy(u8 *p);
void HandleUnitReadData(u8 *p);


#endif
