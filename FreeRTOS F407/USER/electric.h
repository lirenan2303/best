#ifndef __ELECTRIC_H
#define __ELECTRIC_H

typedef enum
{
	ELEACKERROR = 0,          /*从站应答异常*/
	ELECQUERYACK = 0x88,      /*网关数据查询*/
	ELECSOFTQUERYACK = 0x8E,  /*电量板版本号查询*/
	ELEFTPUPDATAACK = 0x9E,   /*电量板FTP升级*/
	ELECPROTOCOL_NULL,        /*保留*/
}ElecProtocolType;

typedef struct
{
	ElecProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
} ElecMessageHandlerMap;

void EleDMA_TxBuff(char *buf, u8 buf_size);
void ElecHandleGWDataQuery(u8 *p);
void ElecHandleSoftVerQuery(u8 *p);
void ElecHandleFTPUpdata(u8 *p);

void ElectricInit(void);

#endif
