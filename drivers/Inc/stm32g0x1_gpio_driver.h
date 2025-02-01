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
    uint8_t GPIO_PinOPType; // Possible values from @GPIO_PIN_OUTPUT_TYPE
    uint8_t GPIO_PinAltFunMode; // Possible values from @GPIO_PIN_ALT_FUN_MODE
}GPIO_PinConfig_t;

typedef struct
{
    // pointer t hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx; // Holds the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig; // Holds the pin configuration settings

}GPIO_Handle_t;


/*
 @GPIO_PIN_NUMBERS
*/
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
* @GPIO_PIN_MODES
*/
#define GPIO_INPUT_MODE         0
#define GPIO_OUTPUT_MODE        1
#define GPIO_ALT_FUNCTION_MODE  2
#define GPIO_ANALOG_MODE        3
#define GPIO_IT_FT_MODE         4 // Falling edge trigger
#define GPIO_IT_RT_MODE         5 // Rising edge trigger
#define GPIO_IT_RFT_MODE        6 // Rising and falling edge trigger

/*
* @GPIO_PIN_OUTPUT_TYPE
*/
#define GPIO_OP_TYPE_PP         0 // push-pull output type
#define GPIO_OP_TYPE_OD         1 // open-drain output type

/*
 @GPIO_PIN_SPEED
*/
#define GPIO_SPEED_VERY_LOW     0
#define GPIO_SPEED_LOW          1
#define GPIO_SPEED_HIGH         2
#define GPIO_SPEED_VERY_HIGH    3

/*
 @GPIO_PIN_PUPD_CONTROL
*/
#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2

/*
 @GPIO_PIN_ALT_FUN_MODE
*/
#define GPIO_AF0                0
#define GPIO_AF1                1
#define GPIO_AF2                2
#define GPIO_AF3                3
#define GPIO_AF4                4
#define GPIO_AF5                5
#define GPIO_AF6                6
#define GPIO_AF7                7


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