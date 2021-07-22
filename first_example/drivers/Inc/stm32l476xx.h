/*
 * stm32l476xx.h
 *
 *  Created on: Jul 21, 2021
 *      Author: fcant
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>
#define __vo volatile



/*MEMORY BASEADDRESS*/
#define BASE_ADD_FLASH				0x08000000U
#define BASE_ADD_SRAM1				0x20000000U //96 Kbytes
#define BASE_ADD_SRAM2				0x10000000U //32Kb up to 0x10008000
#define ROM							0x1FFF0000U //System Memory


/*MAIN BUSES*/
#define PERIPHERAL_BASE				0x40000000U
#define APB1_BASE_ADD				PERIPHERAL_BASE
#define APB2_BASE_ADD				0x40010000U
#define AHB1_BASE_ADD				0x40020000U
#define AHB2_BASE_ADD				0x48000000U

/*GPIO BASE ADDRESS*/
#define GPIOA_BASE_ADD						(AHB2_BASE_ADD+0x0000)
#define GPIOB_BASE_ADD						(AHB2_BASE_ADD+0x0400)
#define GPIOC_BASE_ADD						(AHB2_BASE_ADD+0x0800)
#define GPIOD_BASE_ADD						(AHB2_BASE_ADD+0x0C00)
#define GPIOE_BASE_ADD						(AHB2_BASE_ADD+0x1000)
#define GPIOF_BASE_ADD						(AHB2_BASE_ADD+0x1400)
#define GPIOG_BASE_ADD						(AHB2_BASE_ADD+0x1800)
#define GPIOH_BASE_ADD						(AHB2_BASE_ADD+0x1C00)

/*I2C BASE ADDRESS*/
#define I2C1_BASE_ADD						(APB1_BASE_ADD+0x5400)
#define I2C2_BASE_ADD						(APB1_BASE_ADD+0x5800)
#define I2C3_BASE_ADD						(APB1_BASE_ADD+0x5C00)

/*SPI BASE ADDRESS*/
#define SPI1_BASE_ADD						(APB2_BASE_ADD+0x3000)
#define SPI2_BASE_ADD						(APB1_BASE_ADD+0x3800)
#define SPI3_BASE_ADD						(APB1_BASE_ADD+0x3C00)

/*USART BASE ADDRESS*/
#define USART1_BASE_ADD						(APB2_BASE_ADD+0x3800)
#define USART2_BASE_ADD						(APB1_BASE_ADD+0x4400)
#define USART3_BASE_ADD						(APB1_BASE_ADD+0x4800)

/*UART BASE ADDRESS*/
#define UART4_BASE_ADD						(APB1_BASE_ADD+0x4C00)
#define UART5_BASE_ADD						(APB1_BASE_ADD+0x5000)

/*EXTI BASE ADDRESS*/
#define EXTI_BASE_ADD						(APB2_BASE_ADD+0x0400)
/*SYSCFG BASE ADDRESS*/
#define SYSCFG_BASE_ADD						(APB2_BASE_ADD+0x0000)

#define RCC_BASE_ADD						(AHB1_BASE_ADD+0x1000)


/* Peripheral structure for GPIO*/
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t OPUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFLR;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
	__vo uint32_t ASCR;

}GPIO_RegDef_t;


#define GPIOA 				(GPIO_RegDef*)GPIOA_BASE_ADD
#define GPIOB 				(GPIO_RegDef*)GPIOB_BASE_ADD
#define GPIOC 				(GPIO_RegDef*)GPIOC_BASE_ADD
#define GPIOD 				(GPIO_RegDef*)GPIOD_BASE_ADD
#define GPIOE 				(GPIO_RegDef*)GPIOE_BASE_ADD
#define GPIOF 				(GPIO_RegDef*)GPIOF_BASE_ADD
#define GPIOG 				(GPIO_RegDef*)GPIOG_BASE_ADD

typedef struct
{
	__vo uint32_t CR;				//0x00
	__vo uint32_t ICSCR;			//0x04
	__vo uint32_t CFGR;				//0x08
	__vo uint32_t PLLCFGR;			//0x0C
	__vo uint32_t PLLSAI1_CFGR;		//0x10
	__vo uint32_t PLLSAI2_CFGR;		//0x14
	__vo uint32_t CIER;				//0x18
	__vo uint32_t CIFR;				//0x1C
	__vo uint32_t CICR;				//0x20
	__vo uint32_t RESERVED1;
	__vo uint32_t AHB1RSTR;			//0x28
	__vo uint32_t AHB2RSTR;			//0x2C
	__vo uint32_t AHB3RSTR;			//0x30
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1RSTR1;		//0x38
	__vo uint32_t APB1RSTR2;		//0x3C
	__vo uint32_t APB2RSTR;			//0x40
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1_ENR;			//0x48
	__vo uint32_t AHB2_ENR;			//0x4C
	__vo uint32_t AHB3_ENR;			//0x50
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1ENR1;			//0x58
	__vo uint32_t APB1ENR2;			//0x5C
	__vo uint32_t APB2ENR;			//0x60
	__vo uint32_t RESERVED5;
	__vo uint32_t AHB1SMENR;		//0x68
	__vo uint32_t AHB2SMENR;		//0x6C
	__vo uint32_t AHB3SMENR;		//0x70
	__vo uint32_t RESERVED6;
	__vo uint32_t APB1SM_ENR1;		//0x78
	__vo uint32_t APB1SM_ENR2;		//0x7C
	__vo uint32_t APB2SMENR;		//0x80
	__vo uint32_t RESERVED7;
	__vo uint32_t CCIPR;			//0x88
	__vo uint32_t RESERVED8;
	__vo uint32_t BDCR;				//0x90
	__vo uint32_t CSR;				//0x94
	__vo uint32_t CRRCR;			//0x98
	__vo uint32_t CCIPR2;			//0x9C
}RCC_RegDef_t;

#define RCC 			(RCC_RegDef*)RCC_BASE_ADD


#endif /* INC_STM32L476XX_H_ */
