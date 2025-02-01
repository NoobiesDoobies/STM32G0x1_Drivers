/*
 * stm32g0x1.h
 *
 *  Created on: Feb 1, 2025
 *      Author: carlios
 */

/*
 * base addresses of Flash and SRAM memories
 */

#ifndef INC_STM32G0X1_H_
#define INC_STM32G0X1_H_

#define MAIN_FLASH_BASEADDR					0x08000000U // Unsigned base address of Main Flash Memory
#define SRAM1_BASEADDR						0x20000000U // Unsigned base address of SRAM1
#define ROM									0x1FFF0000U // Unsigned base address of system memory/ROM
#define SRAM								SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE							0x40000000U // Start at TIM2 Peripheral
#define APB1PERIPH_BASE						PERIPH_BASE // starts on 0x4000 0000 ends on 0x4000 A7FF
#define APB2PERIPH_BASE						0x40010000U // Different me
#define AHB1PERIPH_BASE						0x40020000U // starts on 0x4002 0000 ends on 0x4002 63FF
#define IOPORTPERIPH_BASE					0x50000000U // Peripheral addresses containing GPIO A-F pins

/*
 * Base addresses of peripherals which are connected to IOPORT B
 */

#define GPIOA_BASEADDR						(IOPORTPERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR						(IOPORTPERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR						(IOPORTPERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR						(IOPORTPERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR						(IOPORTPERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR						(IOPORTPERIPH_BASE + 0x1400)

/*
 * Base addresses of peripherals which are connected to AHB Bus
 */

#define DMA1_BASEADDR						(AHB1PERIPH_BASE + 0x0000)
#define DMA2_BASEADDR						(AHB1PERIPH_BASE + 0x0400)
#define DMAMUX_BASEADDR						(AHB1PERIPH_BASE + 0x0800)
#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x1000)
#define EXTI_BASEADDR						(AHB1PERIPH_BASE + 0x1800)
#define FLASH_BASEADDR						(AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR						(AHB1PERIPH_BASE + 0x3000)
#define RNG_BASEADDR						(AHB1PERIPH_BASE + 0x5000)
#define AES_BASEADDR						(AHB1PERIPH_BASE + 0x6000)

/*
 * Base addresses of peripherals which are connected to the APB1 Bus
 */

#define TIM2_BASEADDR						(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR						(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR						(APB1PERIPH_BASE + 0x0800)
#define TIM6_BASEADDR						(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR						(APB1PERIPH_BASE + 0x1400)
#define TIM14_BASEADDR						(APB1PERIPH_BASE + 0x2000)
#define RTC_BASEADDR						(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR						(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR						(APB1PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)
#define USART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00)
#define USART5_BASEADDR						(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800)
#define USB_BASEADDR						(APB1PERIPH_BASE + 0x5C00)
#define FDCAN1_BASEADDR						(APB1PERIPH_BASE + 0x6400)
#define FDCAN2_BASEADDR						(APB1PERIPH_BASE + 0x6800)
#define CRS_BASEADDR						(APB1PERIPH_BASE + 0x6C00)
#define PWR_BASEADDR						(APB1PERIPH_BASE + 0x7000)
#define DAC_BASEADDR						(APB1PERIPH_BASE + 0x7400)
#define CEC_BASEADDR						(APB1PERIPH_BASE + 0x7800)
#define LPTIM1_BASEADDR						(APB1PERIPH_BASE + 0x7C00)
#define LPUART1_BASEADDR					(APB1PERIPH_BASE + 0x8000)
#define LPUART2_BASEADDR					(APB1PERIPH_BASE + 0x8400)
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x8800)
#define LPTIM2_BASEADDR						(APB1PERIPH_BASE + 0x9400)
#define USB_RAM1_BASEADDR					(APB1PERIPH_BASE + 0x9800)
#define USB_RAM2_BASEADDR					(APB1PERIPH_BASE + 0x9C00)
#define UCPD1_BASEADDR						(APB1PERIPH_BASE + 0xA000)
#define UCPD2_BASEADDR						(APB1PERIPH_BASE + 0xA400)
#define TAMP_BASEADDR						(APB1PERIPH_BASE + 0xB000)
#define FDCAN_BASEADDR						(APB1PERIPH_BASE + 0xB400)

/*
 * Base addresses of peripherals which are connected to the APB2 Bus
 */
#define SYSCFG_BASEADDR						(APB2PERIPH_BASE + 0x0000)
#define VREFBUF_BASEADDR					(APB2PERIPH_BASE + 0x0030)
#define SYSCFG_ITLINE_BASEADDR				(APB2PERIPH_BASE + 0x0080)
#define COMP_BASEADDR						(APB2PERIPH_BASE + 0x0200)
#define ADC_BASEADDR						(APB2PERIPH_BASE + 0x2400)
#define TIM1_BASEADDR						(APB2PERIPH_BASE + 0x2C00)
#define USART1_BASEADDR						(APB2PERIPH_BASE + 0x3800)
#define USART6_BASEADDR						(APB2PERIPH_BASE + 0x3C00)
#define TIM15_BASEADDR						(APB2PERIPH_BASE + 0x4000)
#define TIM16_BASEADDR						(APB2PERIPH_BASE + 0x4400)
#define TIM17_BASEADDR						(APB2PERIPH_BASE + 0x4800)
#define DBG_BASEADDR						(APB2PERIPH_BASE + 0x5800)

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile uint32_t MODER; 	// GPIO port mode register	// 00: input, 01: output, 10: alt, 11: analog // 0x00 Offset
	volatile uint32_t TYPER; 	// GPIO port output type register	//  0: push-pull, 1: open-drain // 0x04 offset
	volatile uint32_t SPEEDR; 	// GPIO port output speed register // 00: very low, 01: low, 10: high, 11: high speed // 0x08 offset
	volatile uint32_t PUPDR; 	// GPIO port pull-up/pull-down register // 00: No pull-up, pull-down, 01: pull-up, 10: pull-down, 11: reserved
	volatile uint32_t IDR; 		// GPIO port input data register 				// 0x10 offset
	volatile uint32_t ODR; 		// GPIO port output data register 				// 0x14 offset
	volatile uint32_t BSRR; 	// GPIO port bit set/reset register 			// 0x18 offset
	volatile uint32_t LCKR; 	// GPIO port configuration lock register 		// 0x1C offset
	volatile uint32_t AFR[2]; 	// AF[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register
	volatile uint32_t BRR; 		// GPIO port bit reset register 				// 0x28 offset
}GPIO_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)

/*
 * Peripheral register definition structure for RCC
*/

typedef struct
{
	volatile uint32_t CR;					// RCC clock control register
	volatile uint32_t ICSCR;				// RCC internal clock sources calibration register
	volatile uint32_t CFGR;					// RCC clock configuration register
	volatile uint32_t PLLCFGR;				// RCC PLL configuration register
	uint32_t RESERVED1;			// Reserved 0x10 offset
	volatile uint32_t CRRCR;				// RCC clock recovery RC register
	volatile uint32_t CIER;					// RCC clock interrupt enable register
	volatile uint32_t CIFR;					// RCC clock interrupt flag register
	volatile uint32_t CICR;					// RCC clock interrupt clear register
	volatile uint32_t IOPRSTR;				// RCC I/O port reset register
	volatile uint32_t AHBRSTR;				// RCC AHB peripheral reset register
	volatile uint32_t APBRSTR1;				// RCC APB peripheral reset register 1
	volatile uint32_t APBRSTR2;				// RCC APB peripheral reset register 2
	volatile uint32_t IOPENR;				// RCC I/O port enable register
	volatile uint32_t AHBENR;				// RCC AHB peripheral clock enable register
	volatile uint32_t APBENR1;				// RCC APB peripheral clock enable register 1
	volatile uint32_t APBENR2;				// RCC APB peripheral clock enable register 2
	volatile uint32_t IOPSMENR;				// RCC I/O port in sleep mode enable register
	volatile uint32_t AHBSMENR;				// RCC AHB peripheral clock enable in sleep mode register
	volatile uint32_t APBSMENR1;			// RCC APB peripheral clock enable in sleep mode register 1
	volatile uint32_t APBSMENR2;			// RCC APB peripheral clock enable in sleep mode register 2
	volatile uint32_t CCIPR;				// RCC clock configuration register
	volatile uint32_t CCIPR2;				// RCC clock configuration register 2
	volatile uint32_t BDCR;					// RCC backup domain control register
	volatile uint32_t CSR;					// RCC control/status register
}RCC_RegDef_t;

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#endif /* INC_STM32G0X1_H_ */
