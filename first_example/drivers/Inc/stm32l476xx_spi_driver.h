/*
 * stm32l476xx_spi_driver.h
 *
 *  Created on: Oct 4, 2021
 *      Author: franc
 */
#include "stm32l476xx.h"


#ifndef INC_STM32L476XX_SPI_DRIVER_H_
#define INC_STM32L476XX_SPI_DRIVER_H_


/*@SPI_DeviceMode*/
#define SPI_MASTER					1
#define SPI_SLAVE					0
/*@SPI_BusConfig*/
#define SPI_BCONFIG_FD				1
#define SPI_BCONFIG_HD				2
#define SPI_BCONFIG_S_RX			3
/*@SPI_DFF*/
#define SPI_DFF_8BIT				0
#define SPI_CPHA_16BIT				1
/*@SPI_CPHA*/
#define SPI_CPHA_1					1
#define SPI_CPHA_0					0
/*@SPI_CPOL*/
#define SPI_CPOL_1					1
#define SPI_CPOL_0					0
/*@SPI_Speed*/
#define SR_BR_PCLK_DIV_2			0
#define SR_BR_PCLK_DIV_4			1
#define SR_BR_PCLK_DIV_8			2
#define SR_BR_PCLK_DIV_16			3
#define SR_BR_PCLK_DIV_32			4
#define SR_BR_PCLK_DIV_64			5
#define SR_BR_PCLK_DIV_128			6
#define SR_BR_PCLK_DIV_256			7
/*@SPI_DeviceMode*/
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0


typedef struct
{
	uint8_t SPI_DeviceMode;		/*Possible Values from @SPI_DeviceMode*/
	uint8_t SPI_BusConfig; 		/*Possible Values from @SPI_BusConfig*/
	uint8_t SPI_DFF; 			/*Possible Values from @SPI_DFF*/
	uint8_t SPI_CPHA;			/*Possible Values from @SPI_CPHA*/
	uint8_t SPI_CPOL;			/*Possible Values from @SPI_CPOL*/
	uint8_t SPI_SSM;			/*Possible Values from @SPI_SSM*/
	uint8_t SPI_Speed;			/*Possible Values from @SPI_Speed*/

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t* pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;

/*SPI CLCK Control*/
void SPI_PeriCloclControl(SPI_RegDef_t* pSPIx,uint8_t EnorDi);
void SPI_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
void SPI_SSI_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
void SPI_SSOE_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
/*Init De-Init*/
void SPI_Init(SPI_Handle_t * pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/*Data send and Receive*/

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTXBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRXBuffer, uint32_t len);

/*IRQ Handling*/
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_Handling(SPI_Handle_t pSPI_Handle);
void SPI_IRQPRConfig(uint8_t IRQNumber, uint32_t IRQPriority);
#endif /* INC_STM32L476XX_SPI_DRIVER_H_ */
