#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

typedef enum
{
	UNITLIGHTPARAM = 0x82,      /*�Ʋ�������*/
	UNITSTRATEGY = 0x83,        /*�Ʋ�������*/
	UNITREADDATA = 0x86,        /*������������*/
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
