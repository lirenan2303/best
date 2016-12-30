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
	ACKERROR = 0,           /*��վӦ���쳣*/
	GATEPARAM = 0x01,       /*���ز�������*/
	LAMPPARAM = 0x02,       /*�Ʋ�������*/
	LAMPSTRATEGY = 0x03,    /*�Ʋ�������*/
	LAMPDIMMING = 0x04,     /*�Ƶ������*/
	LAMPONOFF = 0x05,       /*�ƿ��ؿ���*/
	READLAMPDATA = 0x06,    /*������������*/
	BRANCHCTRL = 0x07,      /*���ػ�·����*/
	DATAQUERY = 0x08,       /*�������ݲ�ѯ*/
	UNITRUNBACK = 0x0A,     /*������������*/
	TIMEADJUST = 0x0B,      /*Уʱ*/
	VERSIONQUERY = 0x0C,    /*��������汾�Ų�ѯ*/ 
  ELECVERSION = 0x0E,     /*�����ɼ�����汾�Ų�ѯ*/	
	GWADDRQUERY = 0x11,     /*���ص�ַ��ѯ*/
	SETSERVERIPPORT = 0x14, /*���÷�����IP�Ͷ˿�*/
	GATEUPGRADE = 0x15,     /*����Զ������*/
	GPRSQUALITY = 0x17,     /*gprs�ź�ǿ��*/
	SETPARAMLIMIT = 0x21,   /*���ù�ǿ�ȶκ�ʱ��λ��ֵ����*/
	TUNNELSTRATEGY = 0x22,  /*��������ز�������*/
	LUXVALUE = 0x43,        /*���յ���ǿ��ֵ*/
	RESTART = 0x3F,         /*�豸��λ*/
	WGPROTOCOL_NULL,        /*����*/
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
	void (*handlerFunc)(u8 *); //(*ָ�������)(�β��б�)(ָ��ǿ������ת��)
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
