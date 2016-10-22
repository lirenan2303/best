#ifndef __BALLAST_PROTOCOL_H
#define __BALLAST_PROTOCOL_H

#define MAX_LAMP_NUM 1000
#define LAMP_DATA_TEMP_SIZE 50

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


#endif
