/*
 * 01_LedToggle.c
 *
 *  Created on: Feb 1, 2025
 *      Author: carlios
 */


#include "stm32g0x1.h"

void delay(void)
{
	for(uint32_t i=0; i<500*1000;i++);
}

extern void initialise_monitor_handles(void);

int main(void)
{
	initialise_monitor_handles();
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT_MODE;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_InitPin(&GpioLed);


	while(1)
	{
		printf("toggling :D\n");
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
