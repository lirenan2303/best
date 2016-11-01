#ifndef __GATEWAY_PROTOCOL_H
#define __GATEWAY_PROTOCOL_H

typedef enum
{
	ACKERROR = 0,           /*��վӦ���쳣*/
	GATEPARAM = 0x01,       /*���ز�������*/
	LAMPPARAM = 0x02,       /*�Ʋ�������*/
	LAMPSTRATEGY = 0x03,    /*�Ʋ�������*/
	LAMPDIMMING = 0x04,     /*�Ƶ������*/
	LAMPONOFF = 0x05,       /*�ƿ��ؿ���*/
	READDATA = 0x06,        /*������������*/
	LOOPCONTROL = 0x07,     /*���ػ�·����*/
	DATAQUERY = 0x08,       /*�������ݲ�ѯ*/
	VERSIONQUERY = 0x0C,    /*��������汾�Ų�ѯ*/ 
  ELECTRICGATHER = 0x0E,  /*�����ɼ�����汾�Ų�ѯ*/	
	SETPARAMLIMIT = 0x21,   /*���ù�ǿ�ȶκ�ʱ��λ��ֵ����*/
	TUNNELSTRATEGY = 0x22,  /*��������ز�������*/
	GATEUPGRADE = 0x37,     /*����Զ������*/
	TIMEADJUST = 0x42,      /*Уʱ*/
	LUXVALUE = 0x43,        /*���յ���ǿ��ֵ*/
	RESTART = 0x3F,         /*�豸��λ*/
	PROTOCOL_NULL,          /*����*/
}ProtocolType;

typedef struct
{
	ProtocolType type;
	void (*handlerFunc)(u8 *); //(*ָ�������)(�β��б�)(ָ��ǿ������ת��)
} MessageHandlerMap;

void HandleGatewayParam(u8 *p);
void HandleLampParam(u8 *p);
void HandleLampStrategy(u8 *p);
void HandleLampDimmer(u8 *p);
void HandleLampOnOff(u8 *p);
void HandleTunnelStrategy(u8 *p);

void AllParaInit(void);


#endif
