#ifndef __GATEWAY_PROTOCOL_H
#define __GATEWAY_PROTOCOL_H

#include "stm32f4xx.h"

#define ALL_BRANCH  0xFF
#define ALL_SEGMENT 0xFF
#define RANDOM      0x00

#define LAMP_ON     0x30
#define LAMP_OFF    0x31

typedef enum
{
	ACKERROR = 0,           /*从站应答异常*/
	GATEPARAM = 0x01,       /*网关参数下载*/
	LAMPPARAM = 0x02,       /*灯参数下载*/
	LAMPSTRATEGY = 0x03,    /*灯策略下载*/
	LAMPDIMMING = 0x04,     /*灯调光控制*/
	LAMPONOFF = 0x05,       /*灯开关控制*/
	READLAMPDATA = 0x06,    /*读镇流器数据*/
	BRANCHCTRL = 0x07,      /*网关回路控制*/
	DATAQUERY = 0x08,       /*网关数据查询*/
	UNITRUNBACK = 0x0A,     /*单灯自主运行*/
	TIMEADJUST = 0x0B,      /*校时*/
	VERSIONQUERY = 0x0C,    /*网关软件版本号查询*/ 
  ELECVERSION = 0x0E,     /*电量采集软件版本号查询*/	
	GWADDRQUERY = 0x11,     /*网关地址查询*/
	SETSERVERIPPORT = 0x14, /*设置服务器IP和端口*/
	GATEUPGRADE = 0x15,     /*网关远程升级*/
	GPRSQUALITY = 0x17,     /*gprs信号强度*/
	SETPARAMLIMIT = 0x21,   /*设置光强度段和时间段划分点参数*/
	TUNNELSTRATEGY = 0x22,  /*隧道内网关策略下载*/
	LUXVALUE = 0x43,        /*接收到光强度值*/
	RESTART = 0x3F,         /*设备复位*/
	WGPROTOCOL_NULL,        /*保留*/
}WGProtocolType;


typedef struct
{
	unsigned char system_reset_flag;
	unsigned char gsm_reset_flag;
}FlagTypeDef;


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
	
	u8 ConnectFail_Num;
}AlarmParmTypeDef; 

typedef struct
{
	WGProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
} WGMessageHandlerMap;

void CSQ_Reply(char *p);
void branch_state_update(u8 branch, u8 state);
void RTC_TimeToChar(u16 *buf);
ErrorStatus GatewayAddrCheck(u8 *addr_buf);
ErrorStatus GPRS_Protocol_Check(u8 *buf, u8 *BufData_Size);
void GPRS_Protocol_Response(u8 Function, u8 *databuff, u8 DataLength);
void HandleGatewayParam(u8 *p);
void HandleLampParam(u8 *p);
void HandleLampStrategy(u8 *p);
void HandleLampDimmer(u8 *p);
void HandleLampOnOff(u8 *p);
void HandleTunnelStrategy(u8 *p);
void GPRSSendUnitDataFun(u16 *addr_bcd, u8 num, u16 pro_type);
void HandleReadBSNData(u8 *p);
void HandleBranchOnOff(u8 *p);
void HandleGWDataQuery(u8 *p);
void HandleUnitRunBack(u8 *p);
void HandleGWVersQuery(u8 *p);
void HandleElecVersQuery(u8 *p);
void HandleGWAddrQuery(u8 *p);
void HandleSetIPPort(u8 *p);
void HandleSignalQuality(u8 *p);
void HandleRestart(u8 *p);
void HandleAdjustTime(u8 *p);
void HandleGWUpgrade(u8 *P);

void AllParaInit(void);


#endif
