#ifndef __GSM_H
#define __GSM_H

typedef struct {
	char serverIP[16];
	char  serverPORT[6];
}WG_ServerParameterType;

typedef enum
{
	COMMAND_MODE   = 0x00,   
	DATA_MODE = 0x01,   
}GSM_TranModeType;

#define MANAGER_ADDR_LENGTH  10
#define GSM_BUFF_SIZE        200 //与电量队列必须相等
#define GSM_RESTART_TIME     100

void GSMInit(void); 
ErrorStatus GsmStartConnect(void);
ErrorStatus SendMsgToSim(char*cmd, char *ack, u32 waittime);
void SimSendData(u8 *buf,u8 buf_size);
ErrorStatus SwitchToCommand(void);
ErrorStatus SwitchToData(void);
void GsmDMA_TxBuff(char *buf, u8 buf_size);

#endif
