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

#include <stdint.h>

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
	volatile uint32_t OTYPER; 	// GPIO port output type register	//  0: push-pull, 1: open-drain // 0x04 offset
	volatile uint32_t OSPEEDR; 	// GPIO port output speed register // 00: very low, 01: low, 10: high, 11: high speed // 0x08 offset
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

/*
 Clock Enable and Disable Macros for GPIOx peripherals
*/
#define GPIOA_PCLK_EN()			(RCC->IOPENR |= (1 << 0)) // Enable GPIOA port peripheral clock
#define GPIOB_PCLK_EN()			(RCC->IOPENR |= (1 << 1)) // Enable GPIOB port peripheral clock
#define GPIOC_PCLK_EN()			(RCC->IOPENR |= (1 << 2)) // Enable GPIOC port peripheral clock
#define GPIOD_PCLK_EN()			(RCC->IOPENR |= (1 << 3)) // Enable GPIOD port peripheral clock
#define GPIOE_PCLK_EN()			(RCC->IOPENR |= (1 << 4)) // Enable GPIOE port peripheral clock
#define GPIOF_PCLK_EN()			(RCC->IOPENR |= (1 << 5)) // Enable GPIOF port peripheral clock

#define GPIOA_PCLK_DI()			(RCC->IOPENR &= ~(1 << 0)) // Disable GPIOA port peripheral clock
#define GPIOB_PCLK_DI()			(RCC->IOPENR &= ~(1 << 1)) // Disable GPIOB port peripheral clock
#define GPIOC_PCLK_DI()			(RCC->IOPENR &= ~(1 << 2)) // Disable GPIOC port peripheral clock
#define GPIOD_PCLK_DI()			(RCC->IOPENR &= ~(1 << 3)) // Disable GPIOD port peripheral clock
#define GPIOE_PCLK_DI()			(RCC->IOPENR &= ~(1 << 4)) // Disable GPIOE port peripheral clock
#define GPIOF_PCLK_DI()			(RCC->IOPENR &= ~(1 << 5)) // Disable GPIOF port peripheral clock

/*
 Clock Enable and Disable Macros for DMAx peripherals
*/
#define DMA1_PCLK_EN()			(RCC->AHBENR |= (1 << 0)) // Enable DMA1 peripheral clock
#define DMA2_PCLK_EN()			(RCC->AHBENR |= (1 << 1)) // Enable DMA2 peripheral clock

#define DMA1_PCLK_DI()			(RCC->AHBENR &= ~(1 << 0)) // Disable DMA1 peripheral clock
#define DMA2_PCLK_DI()			(RCC->AHBENR &= ~(1 << 1)) // Disable DMA2 peripheral clock

/* 
 Clock Enable and Disable Macros for FLASH peripheral
*/
#define FLASH_PCLK_EN()			(RCC->AHBENR |= (1 << 8)) // Enable FLASH peripheral clock

#define FLASH_PCLK_DI()			(RCC->AHBENR &= ~(1 << 8)) // Disable FLASH peripheral clock

/*
 Clock Enable and Disable Macros for CRC peripheral
*/
#define CRC_PCLK_EN()			(RCC->AHBENR |= (1 << 12)) // Enable CRC peripheral clock

#define CRC_PCLK_DI()			(RCC->AHBENR &= ~(1 << 12)) // Disable CRC peripheral clock

/*
 Clock Enable and Disable Macros for AES peripheral
*/
#define AES_PCLK_EN()			(RCC->AHBENR |= (1 << 16)) // Enable AES peripheral clock

#define AES_PCLK_DI()			(RCC->AHBENR &= ~(1 << 16)) // Disable AES peripheral clock

/*
 Clock Enable and Disable Macros for RNG peripheral
*/
#define RNG_PCLK_EN()			(RCC->AHBENR |= (1 << 18)) // Enable RNG peripheral clock

#define RNG_PCLK_DI()			(RCC->AHBENR &= ~(1 << 18)) // Disable RNG peripheral clock

/*
 Clock Enable and Disable Macros for I2Cx peripherals
*/
#define I2C1_PCLK_EN()			(RCC->APBENR1 |= (1 << 21)) // Enable I2C1 peripheral clock
#define I2C2_PCLK_EN()			(RCC->APBENR1 |= (1 << 22)) // Enable I2C2 peripheral clock
#define I2C3_PCLK_EN()			(RCC->APBENR1 |= (1 << 23)) // Enable I2C3 peripheral clock

#define I2C1_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 21)) // Disable I2C1 peripheral clock
#define I2C2_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 22)) // Disable I2C2 peripheral clock
#define I2C3_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 23)) // Disable I2C3 peripheral clock

/*
 Clock Enable and Disable Macros for SPIx peripherals
*/
#define SPI2_PCLK_EN()			(RCC->APBENR1 |= (1 << 14)) // Enable SPI2 peripheral clock
#define SPI3_PCLK_EN()			(RCC->APBENR1 |= (1 << 15)) // Enable SPI3 peripheral clock

#define SPI2_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 14)) // Disable SPI2 peripheral clock
#define SPI3_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 15)) // Disable SPI3 peripheral clock

/*
 Clock Enable and Disable Macros for USARTx peripherals
*/
#define USART1_PCLK_EN()		(RCC->APBENR2 |= (1 << 14)) // Enable USART1 peripheral clock
#define USART2_PCLK_EN()		(RCC->APBENR1 |= (1 << 17)) // Enable USART2 peripheral clock
#define USART3_PCLK_EN()		(RCC->APBENR1 |= (1 << 18)) // Enable USART3 peripheral clock
#define USART4_PCLK_EN()		(RCC->APBENR1 |= (1 << 19)) // Enable USART4 peripheral clock
#define USART5_PCLK_EN()		(RCC->APBENR1 |= (1 << 8)) // Enable USART5 peripheral clock
#define USART6_PCLK_EN()		(RCC->APBENR1 |= (1 << 9)) // Enable USART6 peripheral clock

#define USART1_PCLK_DI()		(RCC->APBENR2 &= ~(1 << 14)) // Disable USART1 peripheral clock
#define USART2_PCLK_DI()		(RCC->APBENR1 &= ~(1 << 17)) // Disable USART2 peripheral clock
#define USART3_PCLK_DI()		(RCC->APBENR1 &= ~(1 << 18)) // Disable USART3 peripheral clock
#define USART4_PCLK_DI()		(RCC->APBENR1 &= ~(1 << 19)) // Disable USART4 peripheral clock
#define USART5_PCLK_DI()		(RCC->APBENR1 &= ~(1 << 8)) // Disable USART5 peripheral clock
#define USART6_PCLK_DI()		(RCC->APBENR1 &= ~(1 << 9)) // Disable USART6 peripheral clock

/*
 Clock Enable and Disable Macros for TIMx peripherals
*/
#define TIM1_PCLK_EN()			(RCC->APBENR2 |= (1 << 11)) // Enable TIM1 peripheral clock
#define TIM2_PCLK_EN()			(RCC->APBENR1 |= (1 << 0)) // Enable TIM2 peripheral clock
#define TIM3_PCLK_EN()			(RCC->APBENR1 |= (1 << 1)) // Enable TIM3 peripheral clock
#define TIM4_PCLK_EN()			(RCC->APBENR1 |= (1 << 2)) // Enable TIM4 peripheral clock
#define TIM6_PCLK_EN()			(RCC->APBENR1 |= (1 << 4)) // Enable TIM6 peripheral clock
#define TIM7_PCLK_EN()			(RCC->APBENR1 |= (1 << 5)) // Enable TIM7 peripheral clock
#define TIM15_PCLK_EN()			(RCC->APBENR2 |= (1 << 16)) // Enable TIM14 peripheral clock
#define TIM16_PCLK_EN()			(RCC->APBENR2 |= (1 << 17)) // Enable TIM16 peripheral clock
#define TIM17_PCLK_EN()			(RCC->APBENR2 |= (1 << 18)) // Enable TIM17 peripheral clock

#define TIM1_PCLK_DI()			(RCC->APBENR2 &= ~(1 << 11)) // Disable TIM1 peripheral clock
#define TIM2_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 0)) // Disable TIM2 peripheral clock
#define TIM3_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 1)) // Disable TIM3 peripheral clock
#define TIM4_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 2)) // Disable TIM4 peripheral clock
#define TIM6_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 4)) // Disable TIM6 peripheral clock
#define TIM7_PCLK_DI()			(RCC->APBENR1 &= ~(1 << 5)) // Disable TIM7 peripheral clock
#define TIM15_PCLK_DI()			(RCC->APBENR2 &= ~(1 << 16)) // Disable TIM14 peripheral clock
#define TIM16_PCLK_DI()			(RCC->APBENR2 &= ~(1 << 17)) // Disable TIM16 peripheral clock
#define TIM17_PCLK_DI()			(RCC->APBENR2 &= ~(1 << 18)) // Disable TIM17 peripheral clock

/*
 Clock Enable Macros for SYSCFG, COMP, VREFBUF peripheral
*/
#define SYSCFG_PCLK_EN()		(RCC->APBENR2 |= (1 << 0)) // Enable SYSCFG peripheral clock
#define COMP_PCLK_EN()			SYSCFG_PCLK_EN // Enable COMP peripheral clock
#define VREFBUF_PCLK_EN()		SYSCFG_PCLK_EN // Enable VREFBUF peripheral clock

#define SYSCFG_PCLK_DI()		(RCC->APBENR2 &= ~(1 << 0)) // Disable SYSCFG peripheral clock
#define COMP_PCLK_DI()			SYSCFG_PCLK_DI // Disable COMP peripheral clock
#define VREFBUF_PCLK_DI()		SYSCFG_PCLK_DI // Disable VREFBUF peripheral clock

/*
 Clock Enable and Disable Macros for ADC peripheral
*/
#define ADC_PCLK_EN()			(RCC->APBENR2 |= (1 << 20)) // Enable ADC peripheral clock
#define ADC_PCLK_DI()			(RCC->APBENR2 &= ~(1 << 20)) // Disable ADC peripheral clock


/*
 Macros to reset GPIOx peripherals
*/
#define GPIOA_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 0)); (RCC->IOPRSTR &= ~(1 << 0));}while(0) // Reset GPIOA peripheral
#define GPIOB_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 1)); (RCC->IOPRSTR &= ~(1 << 1));}while(0) // Reset GPIOB peripheral
#define GPIOC_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 2)); (RCC->IOPRSTR &= ~(1 << 2));}while(0) // Reset GPIOC peripheral
#define GPIOD_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 3)); (RCC->IOPRSTR &= ~(1 << 3));}while(0) // Reset GPIOD peripheral
#define GPIOE_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 4)); (RCC->IOPRSTR &= ~(1 << 4));}while(0) // Reset GPIOE peripheral
#define GPIOF_REG_RESET()		do{(RCC->IOPRSTR |= (1 << 5)); (RCC->IOPRSTR &= ~(1 << 5));}while(0) // Reset GPIOF peripheral


#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32g0x1_gpio_driver.h"

#endif /* INC_STM32G0X1_H_ */
