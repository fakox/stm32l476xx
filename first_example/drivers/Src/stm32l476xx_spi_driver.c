/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: Oct 4, 2021
 *      Author: franc
 */

#include "stm32l476xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPI_Handle);

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
	temp|=((pSPI_Handle->SPI_Config.SPI_Speed) << 3);
	temp|=((pSPI_Handle->SPI_Config.SPI_SSM) << 9);
	pSPI_Handle->pSPIx->CR1&=~(0x7FFF);
	pSPI_Handle->pSPIx->CR1=temp;

	pSPI_Handle->pSPIx->CR2&=~(0x7FFF);
	pSPI_Handle->pSPIx->CR2=0x1700;
}
void SPI_DeInit(SPI_RegDef_t* pSPIx){

}

/*Enable SPI*/

void SPI_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if (EnorDi==ENABLE ){
		pSPIx->CR1|=(1<<SPI_BP_SPE);
	}
	else{
		pSPIx->CR1&=~(1<<SPI_BP_SPE);
	}

}
void SPI_SSI_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if (EnorDi==ENABLE ){
		pSPIx->CR1|=(1<<SPI_BP_SSI);
	}
	else{
		pSPIx->CR1&=~(1<<SPI_BP_SSI);
	}

}
void SPI_SSOE_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if (EnorDi==ENABLE ){
			pSPIx->CR2|=(1<<SPI_BP_SSOE);
		}
		else{
			pSPIx->CR2&=~(1<<SPI_BP_SSOE);
		}

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
		if((pSPIx->CR2 & (0xF<<8))== 0x0F00){
			pSPIx->DR=*((uint16_t *)pTXBuffer);
			len--;
			len--;
			(uint16_t *)pTXBuffer++;
		}
		else{
			*((uint8_t *)&(pSPIx->DR))=*((uint8_t *)pTXBuffer);
			len--;
			pTXBuffer++;
		}

	}
	SPI_CLR_OVR(pSPIx);

}
void SPI_CLR_OVR(SPI_RegDef_t* pSPIx){
	uint32_t data;
	while(GetFlagStatus(pSPIx,SPI_BP_OVR) == FLAG_SET){
		data=pSPIx->DR;
		data=pSPIx->SR;
	}
	(void)data;

}
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRXBuffer, uint32_t len){

	while(len>0){
			while(GetFlagStatus(pSPIx,SPI_BP_RXNE) == FLAG_RESET);
			if((pSPIx->CR2 & (0xF<<8))== 0x0F00){
				*((uint16_t *)pRXBuffer)=pSPIx->DR;
				len--;
				len--;
				(uint16_t *)pRXBuffer++;
			}
			else{
				*((uint8_t *)pRXBuffer)=*((uint8_t *)&(pSPIx->DR));
				len--;
				pRXBuffer++;
			}

		}

}

/*IRQ Handling*/
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(IRQNumber<=31){
			*NVIC_ISER0|=1<<IRQNumber;

		}
		else if(IRQNumber>31 && IRQNumber<64 ){
			*NVIC_ISER1|=1<<(IRQNumber%32);

		}
		else if(IRQNumber>=64 && IRQNumber<96 ){
			*NVIC_ISER2|=1<<(IRQNumber%64);

		}
		else{

		}
	}
	else{
		if(IRQNumber<=31){
			*NVIC_ICER0|=1<<IRQNumber;

		}
		else if(IRQNumber>=31 && IRQNumber<64 ){
			*NVIC_ICER1|=1<<(IRQNumber%32);

		}
		else if(IRQNumber>=64 && IRQNumber<96 ){
			*NVIC_ICER2|=1<<(IRQNumber%64);

		}
		else{

		}

	}
}
void SPI_IRQPRConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section= IRQNumber%4;
	uint8_t shift_amount=(8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADD+iprx)|=( IRQPriority << shift_amount  );


}


uint8_t SPI_SendData_IT(SPI_Handle_t* pSPIHandle, uint8_t* pTXBuffer, uint32_t len){
	uint8_t state=pSPIHandle->txState;
	if(state!=SPI_BUSY_TX){
		pSPIHandle->pTX_Buffer=pTXBuffer;
		pSPIHandle->txLen=len;
		pSPIHandle->txState=SPI_BUSY_TX;
		pSPIHandle->pSPIx->CR2|=(1<<SPI_BP_TXEIE);

	}
	return state;

}
uint8_t SPI_ReceiveData_IT(SPI_Handle_t* pSPIHandle, uint8_t* pRXBuffer, uint32_t len){
	uint8_t state=pSPIHandle->rxState;
		if(state!=SPI_BUSY_RX){
			pSPIHandle->pRX_Buffer=pRXBuffer;
			pSPIHandle->rxLen=len;
			pSPIHandle->rxState=SPI_BUSY_RX;
			pSPIHandle->pSPIx->CR2|=(1<<SPI_BP_RXNEIE);

		}
		return state;
}

void SPI_IRQHandling(SPI_Handle_t* pSPIHandle){
	//Check for TXE
	if(pSPIHandle->pSPIx->SR|=(1<<SPI_BP_TXE) && pSPIHandle->pSPIx->CR2==(1<<SPI_BP_TXEIE)){
		spi_txe_interrupt_handle(pSPIHandle);
	}
	//Check for RXE
	if(pSPIHandle->pSPIx->SR|=(1<<SPI_BP_RXNE) && pSPIHandle->pSPIx->CR2==(1<<SPI_BP_RXNEIE)){
		spi_rxe_interrupt_handle(pSPIHandle);
	}
	//Check for OVR
	if(pSPIHandle->pSPIx->SR|=(1<<SPI_BP_OVR)){
		spi_ovr_interrupt_handle(pSPIHandle);

	}

}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle){
	if((pSPI_Handle->pSPIx->CR2 & (0xF<<8))== 0x0F00){
		pSPI_Handle->pSPIx->DR=*((uint16_t *)pSPI_Handle->pTX_Buffer);
		pSPI_Handle->txLen--;
		pSPI_Handle->txLen--;
		(uint16_t *)(pSPI_Handle->pTX_Buffer)++;
	}
	else{ //In case of 8 bit , muste be modified to consider X-bit for SMTL476RG
		*((uint8_t *)&(pSPI_Handle->pSPIx->DR))=*((uint8_t *)pSPI_Handle->pTX_Buffer);
		pSPI_Handle->txLen--;
		pSPI_Handle->pTX_Buffer++;
	}
	if(! pSPI_Handle->txLen){
		pSPI_Handle->pSPIx->CR2&=~(1<<SPI_BP_TXEIE);
		pSPI_Handle->pTX_Buffer=NULL;
		pSPI_Handle->txLen=0;
		pSPI_Handle->txState=SPI_READY;
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_TX_CMPLT);

	}



}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPI_Handle){
	if((pSPI_Handle->pSPIx->CR2 & (0xF<<8))== 0x0F00){
		*((uint16_t *)pSPI_Handle->pRX_Buffer)=pSPI_Handle->pSPIx->DR;
		pSPI_Handle->rxLen--;
		pSPI_Handle->rxLen--;
		(uint16_t *)pSPI_Handle->pRX_Buffer++;
	}
	else{
		//In case of 8 bit , muste be modified to consider X-bit for SMTL476RG
		*((uint8_t *)pSPI_Handle->pRX_Buffer)=*((uint8_t *)&(pSPI_Handle->pSPIx->DR));
		pSPI_Handle->rxLen--;
		pSPI_Handle->pRX_Buffer++;
	}
	if(! pSPI_Handle->rxLen){
		pSPI_Handle->pSPIx->CR2&=~(1<<SPI_BP_RXNEIE);
		pSPI_Handle->pRX_Buffer=NULL;
		pSPI_Handle->rxLen=0;
		pSPI_Handle->rxState=SPI_READY;
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_RX_CMPLT);
	}



}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPI_Handle){
	uint8_t state=pSPI_Handle->txState;
	if(state!=SPI_BUSY_TX){
		uint8_t temp;
		temp=pSPI_Handle->pSPIx->DR;
		temp=pSPI_Handle->pSPIx->SR;
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_OVR_CMPLT);
		(void)temp;
	}

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle,uint8_t AppEvent);
