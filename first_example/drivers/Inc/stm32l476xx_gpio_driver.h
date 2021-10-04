/*
 * stm32l476xx_gpio_driver.h
 *
 *  Created on: Jul 22, 2021
 *      Author: fcant
 */

#ifndef INC_STM32L476XX_GPIO_DRIVER_H_
#define INC_STM32L476XX_GPIO_DRIVER_H_
#include "stm32l476xx.h"

/*GPIO definitions*/

#define GPIO_PIN_0 				0
#define GPIO_PIN_1 				1
#define GPIO_PIN_2 				2
#define GPIO_PIN_3 				3
#define GPIO_PIN_4 				4
#define GPIO_PIN_5 				5
#define GPIO_PIN_6 				6
#define GPIO_PIN_7 				7
#define GPIO_PIN_8 				8
#define GPIO_PIN_9 				9
#define GPIO_PIN_10 			10
#define GPIO_PIN_11 			11
#define GPIO_PIN_12 			12
#define GPIO_PIN_13 			13
#define GPIO_PIN_14 			14
#define GPIO_PIN_15				15

/*@GPIO_MODES*/
#define GPIO_MODE_INPUT 		0
#define GPIO_MODE_GEN_OUTPUT 	1
#define GPIO_MODE_ALT 			2
#define GPIO_MODE_ANALOG 		3
#define GPIO_MODE_IT_FT 		4 //Falling Edge detection
#define GPIO_MODE_IT_RT 		5 //Rising Edge Detection
#define GPIO_MODE_IT_RFT 		6// Falling or Rising Detection

/*@GPIO_OPTypes*/
#define GPIO_OPTYPE_PP 			0
#define GPIO_OPTYPE_OD 			1
/*@GPIO_SPEEDS*/
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3
/*@GPIO_PIN_PU_PD_CONF*/
#define GPIO_PIN_NOPU_NOPD		0
#define GPIO_PIN_PULL_UP		1
#define GPIO_PIN_PULL_DOWN		2




typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; 			/*Possible Values from @GPIO_MODES*/
	uint8_t GPIO_PinSpeed; 			/*Possible Values from @GPIO_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;	/*Possible Values from @GPIO_PIN_PU_PD_CONF*/
	uint8_t GPIO_OPType;			/*Possible Values from @GPIO_OPTypes*/
	uint8_t GPIO_PinAltFuncMode;	/*Possible Values from @GPIO_ALTFUNC*/

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*GPIO CLCK Control*/
void GPIO_PeriCloclControl(GPIO_RegDef_t* pGPIOx,uint8_t EnorDi);

/*Init De-Init*/
void GPIO_Init(GPIO_Handle_t * pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and Handling*/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_Handling(uint8_t PinNumber);
void GPIO_IRQPRConfig(uint8_t IRQNumber, uint32_t IRQPriority);



#endif /* INC_STM32L476XX_GPIO_DRIVER_H_ */
