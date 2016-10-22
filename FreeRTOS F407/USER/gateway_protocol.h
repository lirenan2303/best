#ifndef __GATEWAY_PROTOCOL_H
#define __GATEWAY_PROTOCOL_H

typedef enum
{
	ACKERROR = 0,           /*从站应答异常*/
	GATEPARAM = 0x01,       /*网关参数下载*/
	LIGHTPARAM = 0x02,      /*灯参数下载*/
	STRATEGY = 0x03,        /*灯策略下载*/
	DIMMING = 0x04,         /*灯调光控制*/
	LAMPSWITCH = 0x05,      /*灯开关控制*/
	READDATA = 0x06,        /*读镇流器数据*/
	LOOPCONTROL = 0x07,     /*网关回路控制*/
	DATAQUERY = 0x08,       /*网关数据查询*/
	VERSIONQUERY = 0x0C,    /*网关软件版本号查询*/ 
  ELECTRICGATHER = 0x0E,  /*电量采集软件版本号查询*/	
	SETPARAMLIMIT = 0x21,   /*设置光强度段和时间段划分点参数*/
	STRATEGYDOWN = 0x22,    /*隧道内网关策略下载*/
	GATEUPGRADE = 0x37,     /*网关远程升级*/
	TIMEADJUST = 0x42,      /*校时*/
	LUXVALUE = 0x43,        /*接收到光强度值*/
	RESTART = 0x3F,         /*设备复位*/
	PROTOCOL_NULL,          /*保留*/
}ProtocolType;

typedef struct
{
	ProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
} MessageHandlerMap;

void HandleGatewayParam(u8 *p);
void HandleLightParam(u8 *p);
void HandleStrategyDownload(u8 *p);
void HandleLightDimmer(u8 *p);
void HandleLightOnOff(u8 *p);

void AllParaInit(void);


#endif
