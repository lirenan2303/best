#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

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

void HandleUnitLightParam(u8 *p);
void HandleUnitStrategy(u8 *p);
void HandleUnitReadData(u8 *p);


#endif
