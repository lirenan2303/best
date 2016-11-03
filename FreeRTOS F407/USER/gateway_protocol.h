#ifndef __GATEWAY_PROTOCOL_H
#define __GATEWAY_PROTOCOL_H

typedef enum
{
	ACKERROR = 0,           /*从站应答异常*/
	GATEPARAM = 0x01,       /*网关参数下载*/
	LAMPPARAM = 0x02,       /*灯参数下载*/
	LAMPSTRATEGY = 0x03,    /*灯策略下载*/
	LAMPDIMMING = 0x04,     /*灯调光控制*/
	LAMPONOFF = 0x05,       /*灯开关控制*/
	READDATA = 0x06,        /*读镇流器数据*/
	LOOPCONTROL = 0x07,     /*网关回路控制*/
	DATAQUERY = 0x08,       /*网关数据查询*/
	VERSIONQUERY = 0x0C,    /*网关软件版本号查询*/ 
  ELECTRICGATHER = 0x0E,  /*电量采集软件版本号查询*/	
	SETPARAMLIMIT = 0x21,   /*设置光强度段和时间段划分点参数*/
	TUNNELSTRATEGY = 0x22,  /*隧道内网关策略下载*/
	GATEUPGRADE = 0x37,     /*网关远程升级*/
	TIMEADJUST = 0x42,      /*校时*/
	LUXVALUE = 0x43,        /*接收到光强度值*/
	RESTART = 0x3F,         /*设备复位*/
	PROTOCOL_NULL,          /*保留*/
}ProtocolType;

typedef struct
{
	u16 L1_VolHigh;
	u16 L2_VolHigh;
	u16 L3_VolHigh;
	u16 L1_VolLow;
	u16 L2_VolLow;
	u16 L3_VolLow;
	
	u16 L1_CurLow;
	u16 L2_CurLow;
	u16 L3_CurLow;
	u16 Zero_CurLow;
	u16 L1_CurHigh;
	u16 L2_CurHigh;
	u16 L3_CurHigh;
	u16 Zero_CurHigh;
	
	u16 Error_Num;
}AlarmParmTypeDef; 

typedef struct
{
	ProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
} MessageHandlerMap;

void HandleGatewayParam(u8 *p);
void HandleLampParam(u8 *p);
void HandleLampStrategy(u8 *p);
void HandleLampDimmer(u8 *p);
void HandleLampOnOff(u8 *p);
void HandleTunnelStrategy(u8 *p);

void AllParaInit(void);


#endif
