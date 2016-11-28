#ifndef __ELECTRIC_H
#define __ELECTRIC_H

typedef enum
{
	ELEACKERROR = 0,          /*��վӦ���쳣*/
	ELECQUERYACK = 0x88,      /*�������ݲ�ѯ*/
	ELECSOFTQUERYACK = 0x8E,  /*������汾�Ų�ѯ*/
	ELEFTPUPDATAACK = 0x9E,   /*������FTP����*/
	ELECPROTOCOL_NULL,        /*����*/
}ElecProtocolType;

typedef struct
{
	ElecProtocolType type;
	void (*handlerFunc)(u8 *); //(*ָ�������)(�β��б�)(ָ��ǿ������ת��)
} ElecMessageHandlerMap;

void EleDMA_TxBuff(char *buf, u8 buf_size);
void ElecHandleGWDataQuery(u8 *p);
void ElecHandleSoftVerQuery(u8 *p);
void ElecHandleFTPUpdata(u8 *p);

void ElectricInit(void);

#endif
