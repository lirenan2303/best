#ifndef __ELECTRIC_H
#define __ELECTRIC_H

typedef enum
{
	ELEACKERROR = 0,          /*从站应答异常*/
	ELECQUERYACK = 0x88,      /*网关数据查询*/
	ELECSOFTQUERYACK = 0x8E,  /*电量板版本号查询*/
	ELEFTPUPDATAACK = 0x9E,   /*电量板FTP升级*/
	ELERESETACK = 0xBF,       /*电量板复位*/
	ELECPROTOCOL_NULL,        /*保留*/
}ElecProtocolType;

typedef struct
{
	ElecProtocolType type;
	void (*handlerFunc)(u8 *); //(*指针变量名)(形参列表)(指针强制类型转化)
}ElecMessageHandlerMap;

typedef struct
{
	u16 L1_Vol;
	u16 L2_Vol;
	u16 L3_Vol;

	u16 L1_Cur;
	u16 L2_Cur;
	u16 L3_Cur;
	u16 Zero_Cur;

	u8 ConnectFail_Num;
}WG_EleTypeDef; 

typedef struct
{
	u8 cable_fault;
	u8 water_alarm;
	u8 door_open;
	u8 temp_fault;
	
	u8 light_fault;
  u8 cont_off;
	u8 work_cur_high;
	u8 noload_cur_high;
	
	u8 vol_low;
	u8 vol_high;
	u8 power_down;
	u8 end_loss;

	u8 lose_phase;
	u8 pole_fault;
}WG_AlarmFlagDef; 


void WG_DataSample(u8 branch);
void EleDMA_TxBuff(char *buf, u8 buf_size);
void ElecHandleGWDataQuery(u8 *p);
void ElecHandleSoftVerQuery(u8 *p);
void ElecHandleFTPUpdata(u8 *p);
void ElecHandleReset(u8 *p);

void ElectricInit(void);

#endif
