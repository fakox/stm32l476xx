/*
 * stm32l476xx.h
 *
 *  Created on: Jul 21, 2021
 *      Author: fcant
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#define BASE_ADD_FLASH				0x08000000U
#define BASE_ADD_SRAM1				0x20000000U //96 Kbytes
#define BASE_ADD_SRAM2				0x10000000U //32Kb up to 0x10008000
#define ROM							0x1FFF0000U //System Memory

#define PERIPHERAL_BASE				0x40000000U
#define APB1_BASE_ADDRESS			PERIPHERAL_BASE
#define APB2_BASE_ADDRESS			0x40010000U
#define AHB1_BASE_ADDRESS			0x40020000U
#define APB2_BASE_ADDRESS			0x48000000U




#endif /* INC_STM32L476XX_H_ */
