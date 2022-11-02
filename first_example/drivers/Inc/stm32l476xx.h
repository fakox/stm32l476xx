/*
 * stm32l476xx.h
 *
 *  Created on: Jul 21, 2021
 *      Author: fcant
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_
#include <stddef.h>
#include <stdint.h>
#define __vo volatile
#define __weak __attribute__((weak))

#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4					((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5					((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6					((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7					((__vo uint32_t*)0xE000E11C)

#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4					((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5					((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6					((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7					((__vo uint32_t*)0xE000E19C)

#define NVIC_PR_BASE_ADD			((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

#define EXTI0_NVIC					6
#define EXTI1_NVIC					7
#define EXTI2_NVIC					8
#define EXTI3_NVIC					9
#define EXTI4_NVIC					10
#define EXTI5_9_NVIC				23
#define EXTI10_15_NVIC				40
#define EXTI_SPI1_NVIC				35
#define EXTI_SPI2_NVIC				36
#define EXTI_SPI3_NVIC				51

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


#define GPIOBD_TO_PORT(x)	(	(x==GPIOA)?0:\
								(x==GPIOB)?1:\
								(x==GPIOC)?2:\
								(x==GPIOD)?3:\
								(x==GPIOE)?4:\
								(x==GPIOF)?5:\
								(x==GPIOG)?6:7	)




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


typedef struct{
	uint32_t IMR1;
	uint32_t EMR1;
	uint32_t RTSR1;
	uint32_t FTSR1;
	uint32_t SWIER1;
	uint32_t PR1;
	uint32_t RESERVED1;
	uint32_t IMR2;
	uint32_t EMR2;
	uint32_t RTSR2;
	uint32_t FTSR2;
	uint32_t SWIER2;
	uint32_t PR2;
}EXTI_RegDef_t;


#define EXTI				((EXTI_RegDef_t*)EXTI_BASE_ADD)

/*IRQ Numbers for specific MCU*/

#define IRQ_EXTI_0			6
#define IRQ_EXTI_1			7
#define IRQ_EXTI_2			8
#define IRQ_EXTI_3			9
#define IRQ_EXTI_4			10
#define IRQ_EXTI_9_5		23
#define IRQ_EXTI_15_10		40

#define IRQ_PR_NO_1			1
#define IRQ_PR_NO_2			2
#define IRQ_PR_NO_3			3
#define IRQ_PR_NO_4			4
#define IRQ_PR_NO_5			5
#define IRQ_PR_NO_6			6
#define IRQ_PR_NO_7			7
#define IRQ_PR_NO_8			8
#define IRQ_PR_NO_9			9
#define IRQ_PR_NO_10		10
#define IRQ_PR_NO_11		11
#define IRQ_PR_NO_12		12
#define IRQ_PR_NO_13		13
#define IRQ_PR_NO_14		14
#define IRQ_PR_NO_15		15
#define IRQ_PR_NO_16		16
#define IRQ_PR_NO_17		17
#define IRQ_PR_NO_42		42

typedef struct{
	uint32_t MEMRMP;
	uint32_t CFGR1;
	uint32_t EXTICR_1_4[4];
	uint32_t SCSR;
	uint32_t CFGR2;
	uint32_t SWPR;
	uint32_t SKR;
}SYSCFG_RegDef_t;

#define SYSCFG 				((SYSCFG_RegDef_t*)SYSCFG_BASE_ADD)





/* Peripheral structure for SPI*/
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;

}SPI_RegDef_t;

#define SPI1				((SPI_RegDef_t*)SPI1_BASE_ADD)
#define SPI2				((SPI_RegDef_t*)SPI2_BASE_ADD)
#define SPI3				((SPI_RegDef_t*)SPI3_BASE_ADD)

/**************************************************************/
/*               SPI Register bit positions                   */
/**************************************************************/
/*SPI CR1*/
#define SPI_BP_CPHA					0
#define SPI_BP_CPOL					1
#define SPI_BP_MSTR					2
#define SPI_BP_BR					3
#define SPI_BP_SPE					6
#define SPI_BP_LSBFIRST				7
#define SPI_BP_SSI					8
#define SPI_BP_SSM					9
#define SPI_BP_RXONLY				10
#define SPI_BP_CRCL					11
#define SPI_BP_CRCNEXT				12
#define SPI_BP_CRCEN				13
#define SPI_BP_BIDIOE				14
#define SPI_BP_BIDImode				15
/*SP1 CR2*/
#define SPI_BP_RXDMAEN				0
#define SPI_BP_TXDMAEN				1
#define SPI_BP_SSOE					2
#define SPI_BP_NSSP					3
#define SPI_BP_FRF					4
#define SPI_BP_ERRIE				5
#define SPI_BP_RXNEIE				6
#define SPI_BP_TXEIE				7
#define SPI_BP_DS					8
#define SPI_BP_FRX_TH				12
#define SPI_BP_LDMA_RX				13
#define SPI_BP_LDMA_TX				14
/*SP1 SR*/
#define SPI_BP_RXNE					0
#define SPI_BP_TXE					1
#define SPI_BP_CRCERR				4
#define SPI_BP_MODF					5
#define SPI_BP_OVR					6
#define SPI_BP_BSY					7
#define SPI_BP_FRE					8
#define SPI_BP_FRLVL				9
#define SPI_BP_FTLVL				11


/******************************************************************/



#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

#endif /* INC_STM32L476XX_H_ */
