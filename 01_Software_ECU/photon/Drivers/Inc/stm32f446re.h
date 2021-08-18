/*
 * stm32f446re.h
 *
 *  Created on: 17.08.2021
 *  Author: Maximilian Altrichter
 */
#ifndef DRIVERS_INC_STM32F446RE_H_
#define DRIVERS_INC_STM32F446RE_H_
/*
 * base addresses of Flash and SRAM memories
 */


#include <stdint.h>

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR			AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800UL)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00UL)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800UL)

/******peripheral register definition structures******/

/*
 * GPIO
 */

typedef struct
{
	volatile uint32_t MODER;	/*!< GPIO port mode register     					Address offset : 0x00*/
	volatile uint32_t OTYPER;	/*!< GPIO port output type register     			Address offset : 0x04*/
	volatile uint32_t OSPEEDR;	/*!< GPIO port output speed register     			Address offset : 0x08*/
	volatile uint32_t PUPDR;	/*!< GPIO port pull-up/pull-down register     		Address offset : 0x0C*/
	volatile uint32_t IDR;		/*!< GPIO port input data register			    	Address offset : 0x10*/
	volatile uint32_t ODR;		/*!< GPIO port output data register		    		Address offset : 0x14*/
	volatile uint32_t BSRR;		/*!< GPIO port bit set/reset register     			Address offset : 0x18*/
	volatile uint32_t LCKR;		/*!< GPIO port configuration lock register	    	Address offset : 0x1C*/
	volatile uint32_t AFRL;		/*!< GPIO port alternate function low register    	Address offset : 0x20*/
	volatile uint32_t AFRH;		/*!< GPIO port alternate function high register   	Address offset : 0x24*/
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR;			/*!< RCC clock control register										Address offset : 0x00*/
	volatile uint32_t PLLCFGR;  	/*!< RCC PLL configuration register									Address offset : 0x04*/
	volatile uint32_t CFGR;			/*!< RCC clock configuration register								Address offset : 0x08*/
	volatile uint32_t CIR;			/*!< RCC clock interrupt register									Address offset : 0x0C*/
	volatile uint32_t AHB1RSTR; 	/*!< RCC AHB1 peripheral reset register								Address offset : 0x10*/
	volatile uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register								Address offset : 0x14*/
	volatile uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register								Address offset : 0x18*/
	volatile uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register								Address offset : 0x20*/
	volatile uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register								Address offset : 0x24*/
	volatile uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register						Address offset : 0x30*/
	volatile uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register						Address offset : 0x34*/
	volatile uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register						Address offset : 0x38*/
	volatile uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register						Address offset : 0x40*/
	volatile uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register						Address offset : 0x44*/
	volatile uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register	Address offset : 0x50*/
	volatile uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register	Address offset : 0x54*/
	volatile uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register	Address offset : 0x58*/
	volatile uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register	Address offset : 0x60*/
	volatile uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enable in low power mode register	Address offset : 0x64*/
	volatile uint32_t BDCR;			/*!< RCC Backup domain control register								Address offset : 0x70*/
	volatile uint32_t CSR;			/*!< RCC clock control & status register							Address offset : 0x74*/
	volatile uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register					Address offset : 0x80*/
	volatile uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register								Address offset : 0x84*/
	volatile uint32_t PLLSAICFGR;	/*!< RCC PLL configuration register									Address offset : 0x88*/
	volatile uint32_t DCKCFGR;		/*!< RCC dedicated clock configuration register						Address offset : 0x8C*/
	volatile uint32_t CKGATENR;		/*!< RCC clocks gated enable register								Address offset : 0x90*/
	volatile uint32_t DCKCFGR2;		/*!< RCC dedicated clocks configuration register 2					Address offset : 0x94*/
	//TODO : Fill out by DENRUP
}RCC_RegDef_t;

/*
 * peripheral definitions (Peripheral bas addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#include "GPIO.h"

#endif
