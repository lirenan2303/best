#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

#define MAX_LAMP_NUM 1000
#define MAX_BRANCH_NUM 8
#define LAMP_DATA_TEMP_SIZE 50

typedef enum
{
	UNITLIGHTPARAM = 0x82,      /*�Ʋ�������*/
	UNITSTRATEGY = 0x83,        /*�Ʋ�������*/
//	DIMMING = 0x04,         /*�Ƶ������*/
//	LAMPSWITCH = 0x05,      /*�ƿ��ؿ���*/
	UNITREADDATA = 0x86,        /*������������*/
//	UNITUPDATA = 0X2A,  /*������Զ������*/
//	TIMEADJUST = 0x42,      /*Уʱ*/
 	UNITPROTOCOL_NULL,          /*����*/
}UnitProtocolType;

typedef struct
{
	UnitProtocolType type;
	void (*handlerFunc)(u8 *); //(*ָ�������)(�β��б�)(ָ��ǿ������ת��)
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
