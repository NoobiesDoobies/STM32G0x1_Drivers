#include "stm32g0x1_gpio_driver.h"

/*****************************************************
 * @fn                - GPIO_InitPin
 *
 * @brief              - This function initializes a GPIO pin
 *
 * @param[in]          - GPIO_Handle_t *pGPIOHandle -> Pointer to the GPIO handle structure
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_InitPin(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    // 1. Configure the mode of the GPIO pin

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= (GPIO_ANALOG_MODE))
    {
        // The pin is non-interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        // interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FT_MODE)
        {
            // Configure the FTSR
            EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RT_MODE)
        {
            // Configure the RTSR
            EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding FTSR bit
            EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RFT_MODE)
        {
            // Configure the FTSR and RTSR
            EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        

        // 3. Enable the EXTI interrupt delivery using IMR
        EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0;

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing 2 bits
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // 3. Configure the pull-up/pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // 5. Configure the alternate function
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALT_FUNCTION_MODE)
    {
        // Configure the alternate function registers
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;                                             // to determine AFRL or AFRH
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;                                             // to determine the bit/pin position
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));                                           // Clearing 4 bits
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // 4 bits for each pin
    }
}

/*****************************************************
 * @fn                - GPIO_DeInitPin
 *
 * @brief              - This function de-initializes a GPIO pin
 *
 * @param[in]          - GPIO_Handle_t *pGPIOHandle -> Pointer to the GPIO handle structure
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_DeInitPin(GPIO_Handle_t *pGPIOHandle)
{
    if (pGPIOHandle->pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOHandle->pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOHandle->pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOHandle->pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOHandle->pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOHandle->pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
}

/*****************************************************
 * @fn                - GPIO_PeriClockControl
 *
 * @brief              - This function enables or disables the peripheral clock for a GPIO port
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx -> Base address of the GPIO port
 * @param[in]          - uint8_t EnorDi -> ENABLE or DISABLE
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
    }
}

/*****************************************************
 * @fn                - GPIO_ReadFromInputPin
 *
 * @brief              - This function reads the value from a GPIO input pin
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx
 * @param[in]          - uint8_t PinNumber
 *
 * @return             - uint8_t
 *
 * @Note               - none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
    return value;
}

/*****************************************************
 * @fn                - GPIO_ReadFromInputPort
 *
 * @brief              - This function reads the value from a GPIO input port
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx -> Base address of the GPIO port
 *
 * @return             - uint16_t
 *
 * @Note               - none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/*****************************************************
 * @fn                - GPIO_WriteToOutputPin
 *
 * @brief              - This function writes a value to a GPIO output pin
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx -> Base address of the GPIO port
 * @param[in]          - uint8_t PinNumber -> Pin number
 * @param[in]          - uint8_t Value -> Value to be written to the pin
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Write 0 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/*****************************************************
 * @fn                - GPIO_WriteToOutputPort
 *
 * @brief              - This function writes a value to a GPIO output port
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx -> Base address of the GPIO port
 * @param[in]          - uint16_t Value -> Value to be written to the port
 *
 * @return             - none
 *
 * @Note               - none
 *
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*****************************************************
 * @fn                - GPIO_ToggleOutputPin
 *
 * @brief              - This function toggles the value of a GPIO output pin
 *
 * @param[in]          - GPIO_RegDef_t *pGPIOx -> Base address of the GPIO port
 * @param[in]          - uint8_t PinNumber -> Pin number
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*****************************************************
 * @fn                - GPIO_IRQConfig
 *
 * @brief              - This function configures the IRQ for a GPIO pin
 *
 * @param[in]          - uint8_t IRQNumber -> IRQ number
 * @param[in]          - uint8_t EnorDi -> ENABLE or DISABLE
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
}

/*****************************************************
 * @fn                - GPIO_IRQPriorityConfig
 *
 * @brief              - This function configures the priority of the IRQ for a GPIO pin
 *
 * @param[in]          - uint8_t IRQNumber
 * @param[in]          - uint32_t IRQPriority
 *
 * @return             - none
 *
 * @Note               - none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
}
