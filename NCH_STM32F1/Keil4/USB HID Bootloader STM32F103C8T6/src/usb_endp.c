/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_istr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t rxBuff[64];
uint8_t txBuff[64];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void KT_Reset(void);
void KT_Erase(void);
void KT_WriteFlash(void);
void KT_ReadFlash(void);

void EP1_OUT_Callback(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	
	USB_SIL_Read(EP1_OUT, rxBuff);
	switch(rxBuff[0]) {
	case 0:
		KT_Erase();
		break;
	case 1:
		KT_WriteFlash();
		break;
	case 2:
		KT_ReadFlash();
		break;
	case 3:
		KT_Reset();
		break;
	}
	
	SetEPRxStatus(ENDP1, EP_RX_VALID);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void KT_Erase(void) {
	//nhan gia tri va xoa page can thiet
	uint32_t u32Data;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	u32Data=rxBuff[4];
	u32Data<<=8;
	u32Data+=rxBuff[3];
	u32Data<<=8;
	u32Data+=rxBuff[2];
	u32Data<<=8;
	u32Data+=rxBuff[1];
	if(u32Data>=0x08002000) {
		FLASH_ErasePage(u32Data);
		__NOP();
		__NOP();
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

void KT_Reset(void) {
	/*
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	*/
	NVIC_SystemReset();
}

void KT_WriteFlash(void) {
	//ghi 1 DWORD
	uint32_t u32Data, u32Addr, i;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	u32Addr=rxBuff[4];
	u32Addr<<=8;
	u32Addr+=rxBuff[3];
	u32Addr<<=8;
	u32Addr+=rxBuff[2];
	u32Addr<<=8;
	u32Addr+=rxBuff[1];
	if(u32Addr>=0x08002000) {
		for(i=0; i<8; ++i) {
			u32Data=rxBuff[i*4+8];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+7];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+6];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+5];
			
			FLASH_ProgramWord(u32Addr, u32Data);
			
			__NOP();
			__NOP();
			
			u32Addr+=4;
		}
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

void KT_ReadFlash(void) {
	//ghi 1 DWORD
	uint32_t u32Data, u32Addr, i;
	u32Addr=rxBuff[4];
	u32Addr<<=8;
	u32Addr+=rxBuff[3];
	u32Addr<<=8;
	u32Addr+=rxBuff[2];
	u32Addr<<=8;
	u32Addr+=rxBuff[1];
	if(u32Addr>=0x08002000) {
		for(i=0; i<16; ++i) {
			u32Data=*((uint32_t*)u32Addr);
			txBuff[i*4]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+1]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+2]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+3]=(uint8_t)u32Data;
			u32Addr+=4;
		}
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  //PrevXferComplete = 1;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

