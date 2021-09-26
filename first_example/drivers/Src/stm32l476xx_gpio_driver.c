/*
 * stm32l476xx_gpio_driver.c
 *
 *  Created on: Jul 22, 2021
 *      Author: fcant
 */

#include "stm32l476xx_gpio_driver.h"

/*GPIO CLCK Control*/
/******************************************************************
* @fn			- GPIO_PeriCloclControl
*
* @brief 		- Enables or Disable Peripheral clock for the given port
*
* @param[in] 	- Base Address of the GPIO
* @param[in] 	- Enable or disable option could be ENABLE or DISABLE
*
* @return 		- none
*
* @Note 		- none
*
*/

void GPIO_PeriCloclControl(GPIO_RegDef_t* pGPIOx,uint8_t EnorDi)
{
	if (EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}
		else{
		//Not a port
		}
	}
}

/*Init De-Init*/

/******************************************************************
* @fn			- GPIO_Init
*
* @brief 		- Configure GPIO registers for the given GPIOHandle
*
* @param[in] 	- GPIO Handle
*
* @return 		- none
*
* @Note 		- none
*
*/
void GPIO_Init(GPIO_Handle_t * pGPIO_Handle){

	uint32_t temp=0;

	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER&=~((0x3)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIO_Handle->pGPIOx->MODER|=temp;
		temp=0;

	}
	else{

		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			EXTI->FTSR1|=1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
			EXTI->RTSR1 &= ~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
			EXTI->RTSR1 |= 1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
			EXTI->FTSR1 &= ~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
			EXTI->FTSR1|=1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
			EXTI->RTSR1|=1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;

		}
		else{

		}

		temp=(GPIO_MODE_INPUT)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER&=~((0x3)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIO_Handle->pGPIOx->MODER|=temp;
		temp=0;

		uint8_t portcode= GPIOBD_TO_PORT(pGPIO_Handle->pGPIOx);
		uint8_t index=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)%4;

		SYSCFG->EXTICR_1_4[index]=portcode<<(temp*4);
		SYSCFG_PCLK_EN();

		EXTI->IMR1|=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);



	}

	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDER&=~((0x3)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIO_Handle->pGPIOx->OSPEEDER|=temp;
	temp=0;

	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_OPType)<<((pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OTYPER&=~((0x1)<<((pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIO_Handle->pGPIOx->OTYPER|=temp;
	temp=0;

	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OPUPDR&=~((0x3)<<(2*(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIO_Handle->pGPIOx->OPUPDR|=temp;
	temp=0;

	uint8_t temp1=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/8;
	uint8_t temp2=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber%8;

	pGPIO_Handle->pGPIOx->AFR[temp1]&=~((0xF)<<(4*temp2));
	pGPIO_Handle->pGPIOx->AFR[temp1]|=((pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFuncMode)<<(4*temp2));

}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
			if(pGPIOx==GPIOA){
				GPIOA_RST();
			}
			else if(pGPIOx==GPIOB){
				GPIOB_RST();
			}
			else if(pGPIOx==GPIOC){
				GPIOC_RST();
			}
			else if(pGPIOx==GPIOD){
				GPIOD_RST();
			}
			else if(pGPIOx==GPIOE){
				GPIOE_RST();
			}
			else if(pGPIOx==GPIOF){
				GPIOF_RST();
			}
			else if(pGPIOx==GPIOG){
				GPIOG_RST();
			}
			else if(pGPIOx==GPIOH){
				GPIOH_RST();
			}
			else{
			//Not a port
			}

}

/*Read and Write*/

/******************************************************************
* @fn			- GPIO_ReadFromInputPin
*
* @brief 		- Returns value from specific PinNumber of Port
*
* @param[in] 	- GPIO Port
* @param[in] 	- Pin Number
*
* @return 		- Value of the PinNumber of port
*
* @Note 		- none
*
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){

	uint8_t value;

	value= (uint8_t)((pGPIOx->IDR)>>PinNumber) & 0x00000001;

	return value;

}

/******************************************************************
* @fn			- GPIO_ReadFromInputPort
*
* @brief 		- Returns value from specific Port
*
* @param[in] 	- GPIO Port
*
* @return 		- Value of port
*
* @Note 		- none
*
*/
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){

	uint8_t value;

	value= (uint8_t)(pGPIOx->IDR);

	return value;
}

/******************************************************************
* @fn			- GPIO_WriteToOutputPin
*
* @brief 		- Write value from specific PinNumber of Port
*
* @param[in] 	- GPIO Port
* @param[in] 	- Pin Number
*
* @Note 		- none
*
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,uint8_t value){
	if(value==SET){
		pGPIOx->ODR|=(0x1<<PinNumber);
	}
	else if(value==RESET){
		pGPIOx->ODR&=~(0x1<<PinNumber);
	}
	else{
		//Something
	}

}
/******************************************************************
* @fn			- GPIO_WriteToOutputPort
*
* @brief 		- Write value to an specific Port
*
* @param[in] 	- GPIO Port
*
* @Note 		- none
*
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx,uint16_t value){
	pGPIOx->ODR=value;
}

/******************************************************************
* @fn			- GPIO_ToggleOutputPin
*
* @brief 		- Toggle pin value.
*
* @param[in] 	- GPIO Port
* @param[in] 	- Pin Number
*
* @Note 		- none
*
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);
}





/*IRQ Configuration and Handling*/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void GPIO_IRQPRConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section= IRQNumber%4;
	uint8_t shift_amount=(8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADD+iprx)|=( IRQPriority << shift_amount  );


}
void GPIO_Handling(uint8_t PinNumber){
	if(EXTI->PR1&(1<<PinNumber)){
		EXTI->PR1|=(1<<PinNumber);
	}


}
