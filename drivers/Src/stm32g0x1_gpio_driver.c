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
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
    }else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }else if(pGPIOx == GPIOF)
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
    return 0;
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
    return 0;
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
