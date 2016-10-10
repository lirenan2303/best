#ifndef __GSM_H
#define __GSM_H

typedef struct {
	char serverIP[16];
	char  serverPORT[6];
}WG_ServerParameterType;

#define MANAGER_ADDR_LENGTH  10

void GSMInit(void); 
ErrorStatus GsmStartConnect(void);
ErrorStatus SendMsgToSim(char*cmd, char *ack, u32 waittime);
void SimSendData(u8 *buf,u8 buf_size);

#endif
