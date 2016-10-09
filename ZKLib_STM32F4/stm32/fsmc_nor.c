#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "fsmc_nor.h"
#include "stdio.h"

/*******************************************************************************
* Function Name  : FSMC_NOR_Init
* Description    : Configures the FSMC and GPIOs to interface with the NOR memory.
*                  This function must be called before any write/read operation
*                  on the NOR.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NOR_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |
						             RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��PD,PE,PF,PGʱ��  
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);//ʹ��FSMCʱ�� 

	/*-- GPIO Configuration ------------------------------------------------------*/
	/* NOR Data lines configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
								                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
								                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |
								                GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* NOR Address lines configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |   //A0,A1,A2,A3,A4,A5,
								                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
																GPIO_Pin_14 | GPIO_Pin_15;//A6,A7,A8,A9
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |    //A10,A11,A12,A13,A14,A15,
								                GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;  //A16,A17,A18
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;  //A19,A20,A21  ��ַ��û����ô��
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* NOE and NWE configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* NE2 configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	/*GPIO����ӳ��*/
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource11,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);
 
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource3,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource4,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource2,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource3,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource4,GPIO_AF_FSMC); 
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource5,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource13,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource14,GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource15,GPIO_AF_FSMC);

  GPIO_PinAFConfig(GPIOG,GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource1,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource2,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource3,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_FSMC);

	/*--------------FSMC ���� �洢����������------------------------------*/
  readWriteTiming.FSMC_AddressSetupTime = 0x0B;	      //��ַ����ʱ�䣨ADDSET�� 1��HCLKʱ�� 6*1 = 6ns
  readWriteTiming.FSMC_AddressHoldTime = 0x00;	      //��ַ����ʱ�䣨ADDHLD�� 
  readWriteTiming.FSMC_DataSetupTime = 0x12;		      //���ݱ���ʱ�䣨DATAST�� Ϊ8��HCLK 6*8=48ns	 	 
  readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;  //���߻ָ�ʱ��x00
  readWriteTiming.FSMC_CLKDivision = 0x00;            //ʱ�ӷ�Ƶ���� 
  readWriteTiming.FSMC_DataLatency = 0x00;            //���ݲ���ʱ��
  readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_B;//FSMC NOR������ʱ��

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;   //ʹ����FSMC��BANK1���Ӱ��2
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;  //��ֹ��ַ�����߸���
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;  //�洢������ΪNor
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;  //�洢�����ݿ��Ϊ16λ
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;  //�ر�ͻ��ģʽ����
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low; //�ȴ��ź����ȼ���ֻ����ʹ��ͻ������ģʽ����Ч
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;  //�ر�Wrapped burst access mode��ֻ����ʹ��ͻ������ģʽ����Ч
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  //�ȴ��ź����ã�ֻ����ʹ��ͻ������ģʽ����Ч
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;  //ʹ�����BANK��д����
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;  //ʹ��/�رյȴ���Ϣ���ã�ֻ��ʹ��ͻ������ģʽ����Ч
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;  //�ر�Extend Mode
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; //�ر�Write Burst Mode 
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //������ʱ�����
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &readWriteTiming;  //������ʱ�����

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/* Enable FSMC Bank1_NOR Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE); 
}

void FSMC_NOR_ReadID(NOR_IDTypeDef* NOR_ID)
{
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x0090);

	NOR_ID->Manufacturer_Code = *(unsigned short *) ADDR_SHIFT(0x0000);
	NOR_ID->Device_Code1 = *(unsigned short *) ADDR_SHIFT(0x0001);
	NOR_ID->Device_Code2 = *(unsigned short *) ADDR_SHIFT(0x000E);
	NOR_ID->Device_Code3 = *(unsigned short *) ADDR_SHIFT(0x000F);
}

NOR_Status FSMC_NOR_EraseSector(u32 SectorAddr)
{
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x0080);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE((Bank1_NOR2_ADDR + SectorAddr), 0x50);

	return (FSMC_NOR_GetStatus(SectorErase_Timeout));
}

NOR_Status FSMC_NOR_EraseBlock(u32 BlockAddr)
{
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x0080);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE((Bank1_NOR2_ADDR + BlockAddr), 0x30);

  return (FSMC_NOR_GetStatus(BlockErase_Timeout));
}

NOR_Status FSMC_NOR_EraseChip(void)
{
  NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x0080);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x0010);

  return (FSMC_NOR_GetStatus(ChipErase_Timeout));
}

NOR_Status FSMC_NOR_WriteHalfWord(u32 WriteAddr, u16 Data)
{
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
	NOR_WRITE(ADDR_SHIFT(0x0555), 0x00A0);
  NOR_WRITE((Bank1_NOR2_ADDR + WriteAddr), Data);

  return (FSMC_NOR_GetStatus(Program_Timeout));
}

NOR_Status FSMC_NOR_WriteBuffer(const u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite)
{
  NOR_Status status = NOR_ONGOING; 

  do
  {
    /* Transfer data to the memory */
    status = FSMC_NOR_WriteHalfWord(WriteAddr, *pBuffer++);
    WriteAddr = WriteAddr + 2;
    NumHalfwordToWrite--;
  }
  while((status == NOR_SUCCESS) && (NumHalfwordToWrite != 0));
  
  return (status); 
}

u16 FSMC_NOR_ReadHalfWord(u32 ReadAddr)
{
  NOR_WRITE(ADDR_SHIFT(0x005555), 0x00AA); 
  NOR_WRITE(ADDR_SHIFT(0x002AAA), 0x0055);  
  NOR_WRITE((Bank1_NOR2_ADDR + ReadAddr), 0x00F0 );

  return (*(vu16 *)((Bank1_NOR2_ADDR + ReadAddr)));
}

void FSMC_NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead)
{
  NOR_WRITE(ADDR_SHIFT(0x05555), 0x00AA);
  NOR_WRITE(ADDR_SHIFT(0x02AAA), 0x0055);
  NOR_WRITE((Bank1_NOR2_ADDR + ReadAddr), 0x00F0);

  for(; NumHalfwordToRead != 0x00; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a Halfword from the NOR */
    *pBuffer++ = *(vu16 *)((Bank1_NOR2_ADDR + ReadAddr));
    ReadAddr = ReadAddr + 2; 
  }  
}

NOR_Status FSMC_NOR_ReturnToReadMode(void)
{
  NOR_WRITE(Bank1_NOR2_ADDR, 0x00F0);
  return (NOR_SUCCESS);
}

NOR_Status FSMC_NOR_Reset(void)
{
  NOR_WRITE(ADDR_SHIFT(0x00555), 0x00AA);
	NOR_WRITE(ADDR_SHIFT(0x002AA), 0x0055);
	NOR_WRITE(Bank1_NOR2_ADDR, 0x00F0);

  return (NOR_SUCCESS);
}

NOR_Status FSMC_NOR_GetStatus(u32 Timeout)
{ 
  u16 val1 = 0x00, val2 = 0x00;
  NOR_Status status = NOR_ONGOING; 

  /* Get the NOR memory operation status -------------------------------------*/
  while((Timeout != 0x00) && (status != NOR_SUCCESS))
  {
    Timeout--;

	/* Read DQ6 */
    val1 = *(vu16 *)(Bank1_NOR2_ADDR);
    val2 = *(vu16 *)(Bank1_NOR2_ADDR);

    /* If DQ6 did not toggle between the two reads then return NOR_Success */
    if((val1 & 0x0040) == (val2 & 0x0040)) 
        return NOR_SUCCESS;

    val1 = *(vu16 *)(Bank1_NOR2_ADDR);
    val2 = *(vu16 *)(Bank1_NOR2_ADDR);
    
    if((val1 & 0x0040) == (val2 & 0x0040)) 
        return NOR_SUCCESS;
  }

  if(Timeout == 0x00)
      status = NOR_TIMEOUT;

  /* Return the operation status */
  return (status);
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
