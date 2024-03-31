/*
 * 002ButtonPressed.c
 *
 *  Created on: Feb 25, 2024
 *      Author: Rakesh Ahuja
 */

#include "stm32f407xx_gpio_driver.h"



void delay()
{
	for(uint32_t i = 0; i<500000/2; i++);
}
int main()
{
	GPIO_Handle_t gpioSwitch, gpioLed;

	//Switch
	gpioSwitch.pGPIOx = GPIOA;
	gpioSwitch.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioSwitch.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioSwitch.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioSwitch.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIO_PeriClock_Control(gpioSwitch.pGPIOx, ENABLE);
	GPIO_Init(&gpioSwitch);

	//Output
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_PeriClock_Control(gpioLed.pGPIOx, ENABLE);
	GPIO_Init(&gpioLed);


	while(1)
	{
		if(GPIO_ReadFromPin(gpioSwitch.pGPIOx, gpioSwitch.GPIO_PinConfig.GPIO_PinNumber) == SET)
		{
			delay();
			GPIO_Toggle(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQ_Handling(GPIO_PIN_NO_0);

	if(*(NVIC_ISPR1) & ( 1 << GPIO_PIN_NO_0))
	{
		delay();
//		GPIO_Toggle(pGPIOx, PinNumber);
	}
}
