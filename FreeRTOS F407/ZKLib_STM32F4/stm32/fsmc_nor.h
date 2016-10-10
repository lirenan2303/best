/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : fsmc_nor.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Header for fsmc_nor.c file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FSMC_NOR_H
#define __FSMC_NOR_H

#define Bank1_NOR2_ADDR       ((u32)0x64000000)
 		
#define SectorErase_Timeout   ((u32)0x01900000)
#define BlockErase_Timeout    ((u32)0x01900000)
#define ChipErase_Timeout     ((u32)0x09000000)
#define Program_Timeout       ((u32)0x00003200)

#define ADDR_SHIFT(A) (Bank1_NOR2_ADDR + (2 * (A)))
#define NOR_WRITE(Address, Data)  (*(vu16 *)(Address) = (Data))

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
	short Manufacturer_Code;
	short Device_Code1;
	short Device_Code2;
	short Device_Code3;
} NOR_IDTypeDef;

typedef enum
{
  NOR_SUCCESS = 0,
  NOR_ONGOING,
  NOR_ERROR,
  NOR_TIMEOUT
}NOR_Status;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FSMC_NOR_Init(void);
void FSMC_NOR_ReadID(NOR_IDTypeDef* NOR_ID);
NOR_Status FSMC_NOR_EraseSector(u32 SectorAddr);
NOR_Status FSMC_NOR_EraseBlock(u32 BlockAddr);
NOR_Status FSMC_NOR_EraseChip(void);
NOR_Status FSMC_NOR_WriteHalfWord(u32 WriteAddr, u16 Data);
NOR_Status FSMC_NOR_WriteBuffer(const u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
NOR_Status FSMC_NOR_ProgramBuffer(u16* pBuffer, u32 WriteAddr, u32 NumHalfwordToWrite);
u16 FSMC_NOR_ReadHalfWord(u32 ReadAddr);
void FSMC_NOR_ReadBuffer(u16* pBuffer, u32 ReadAddr, u32 NumHalfwordToRead);
NOR_Status FSMC_NOR_ReturnToReadMode(void);
NOR_Status FSMC_NOR_Reset(void);
NOR_Status FSMC_NOR_GetStatus(u32 Timeout);

#endif /* __FSMC_NOR_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
