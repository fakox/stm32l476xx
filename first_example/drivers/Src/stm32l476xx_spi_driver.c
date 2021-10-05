/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: Oct 4, 2021
 *      Author: franc
 */

#include "stm32l476xx_spi_driver.h"

void SPI_PeriCloclControl(SPI_RegDef_t* pSPIx,uint8_t EnorDi){
	if (EnorDi==ENABLE){
			if(pSPIx==SPI1){
				SPI1_PLCK_EN();
			}
			else if(pSPIx==SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx==SPI3){
				SPI3_PCLK_EN();
			}
			else{
			//Not a port
			}
		}
}
/*Init De-Init*/
void SPI_Init(SPI_Handle_t * pSPI_Handle){
	uint32_t temp=0;

	temp|=((pSPI_Handle->SPI_Config.SPI_DeviceMode) << 2);
	if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BCONFIG_FD){
		temp&=~(1<<15);
	}else if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BCONFIG_HD){
		temp|=(1<<15);

	}else if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BCONFIG_S_RX){
		temp&=~(1<<15);
		temp|=(1<<10);
	}
	else{

	}
	temp|=((pSPI_Handle->SPI_Config.SPI_CPHA) << 0);
	temp|=((pSPI_Handle->SPI_Config.SPI_CPOL) << 1);
	temp|=((pSPI_Handle->SPI_Config.SPI_DFF) << 11);
	temp|=((pSPI_Handle->SPI_Config.SPI_Speed) << 3);

	pSPI_Handle->pSPIx->CR1=temp;
}
void SPI_DeInit(SPI_RegDef_t* pSPIx){

}

/*Data send and Receive*/

uint8_t GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FLAGNAME){
	if((pSPIx->SR & (1<<FLAGNAME))>0){
		return FLAG_SET;
	}
	else{
		return FLAG_RESET;
	}
}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTXBuffer, uint32_t len){
	while(len>0){
		while(GetFlagStatus(pSPIx,SPI_BP_TXE) == FLAG_RESET);
		if((pSPIx->CR1 & (1<<11))!= 0){
			pSPIx->DR=*((uint16_t *)pTXBuffer);
			len--;
			len--;
			(uint16_t *)pTXBuffer++;
		}
		pSPIx->DR=*pTXBuffer;
		len--;
		(uint8_t *)pTXBuffer++;
	}

}
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRXBuffer, uint32_t len);

/*IRQ Handling*/
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_Handling(SPI_Handle_t pSPI_Handle);
void SPI_IRQPRConfig(uint8_t IRQNumber, uint32_t IRQPriority);
