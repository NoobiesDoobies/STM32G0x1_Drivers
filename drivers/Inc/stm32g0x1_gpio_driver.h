/*
 * stm32g0x1.h
 *
 *  Created on: Feb 1, 2025
 *      Author: carlios
 */

#ifndef STM32G0X1_GPIO_DRIVER_H
#define STM32G0X1_GPIO_DRIVER_H

#include <stdint.h>

#include "stm32g0x1.h"

typedef struct 
{
    uint8_t GPIO_PinNumber; // Possible values from @GPIO_PIN_NUMBERS
    uint8_t GPIO_PinMode; // Possible values from @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed; // Possible values from @GPIO_PIN_SPEED
    uint8_t GPIO_PinPuPdControl; // Possible values from @GPIO_PIN_PUPD_CONTROL
    uint8_t GPIO_PinOPType; // Possible values from @GPIO_PIN_OP_TYPE
    uint8_t GPIO_PinAltFunMode; // Possible values from @GPIO_PIN_ALT_FUN_MODE
}GPIO_PinConfig_t;

typedef struct
{
    // pointer t hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx; // Holds the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig; // Holds the pin configuration settings

}GPIO_Handle_t;

/*
 Init and De-Init
*/
void GPIO_InitPin(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInitPin(GPIO_Handle_t *pGPIOHandle);
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 Data read and write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
    IRQ Configuration and ISR Handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif