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
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;
	__vo uint32_t ASCR;

}GPIO_RegDef_t;


#define GPIOA 				(GPIO_RegDef_t*)GPIOA_BASE_ADD
#define GPIOB 				(GPIO_RegDef_t*)GPIOB_BASE_ADD
#define GPIOC 				(GPIO_RegDef_t*)GPIOC_BASE_ADD
#define GPIOD 				(GPIO_RegDef_t*)GPIOD_BASE_ADD
#define GPIOE 				(GPIO_RegDef_t*)GPIOE_BASE_ADD
#define GPIOF 				(GPIO_RegDef_t*)GPIOF_BASE_ADD
#define GPIOG 				(GPIO_RegDef_t*)GPIOG_BASE_ADD
#define GPIOH 				(GPIO_RegDef_t*)GPIOH_BASE_ADD

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

#define RCC 			((RCC_RegDef_t*)RCC_BASE_ADD)




/*Macros for enabling PERIPHERAL CLK*/

#define GPIOA_PCLK_EN()		(RCC->AHB2_ENR|=(1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB2_ENR|=(1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB2_ENR|=(1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB2_ENR|=(1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB2_ENR|=(1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB2_ENR|=(1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB2_ENR|=(1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB2_ENR|=(1<<7))


#define I2C3_PCLK_EN()		(RCC->APB1ENR1|=(1<<23))
#define I2C2_PCLK_EN()		(RCC->APB1ENR1|=(1<<22))
#define I2C1_PCLK_EN()		(RCC->APB1ENR1|=(1<<21))
#define SPI2_PCLK_EN()		(RCC->APB1ENR1|=(1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR1|=(1<<15))
#define UART5_PCLK_EN()		(RCC->APB1ENR1|=(1<<20))
#define UART4_PCLK_EN()		(RCC->APB1ENR1|=(1<<19))
#define USART3_PCLK_EN()	(RCC->APB1ENR1|=(1<<18))
#define USART2_PCLK_EN()	(RCC->APB1ENR1|=(1<<17))

#define SPI1_PLCK_EN()		(RCC->APB2ENR|=(1<<12))
#define USART1_PCLK_EN()	(RCC->APB2ENR|=(1<<14))
#define USART1_PCLK_EN()	(RCC->APB2ENR|=(1<<14))
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR|=(1<<0))


/*De-Init functions*/
#define GPIOA_RST()			do{(RCC->AHB2RSTR|=(1<<0));(RCC->AHB2RSTR&=~(1<<0));}while(0)
#define GPIOB_RST()			do{(RCC->AHB2RSTR|=(1<<1));(RCC->AHB2RSTR&=~(1<<1));}while(0)
#define GPIOC_RST()			do{(RCC->AHB2RSTR|=(1<<2));(RCC->AHB2RSTR&=~(1<<2));}while(0)
#define GPIOD_RST()			do{(RCC->AHB2RSTR|=(1<<3));(RCC->AHB2RSTR&=~(1<<3));}while(0)
#define GPIOE_RST()			do{(RCC->AHB2RSTR|=(1<<4));(RCC->AHB2RSTR&=~(1<<4));}while(0)
#define GPIOF_RST()			do{(RCC->AHB2RSTR|=(1<<5));(RCC->AHB2RSTR&=~(1<<5));}while(0)
#define GPIOG_RST()			do{(RCC->AHB2RSTR|=(1<<6));(RCC->AHB2RSTR&=~(1<<6));}while(0)
#define GPIOH_RST()			do{(RCC->AHB2RSTR|=(1<<7));(RCC->AHB2RSTR&=~(1<<7));}while(0)


#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32L476XX_H_ */
