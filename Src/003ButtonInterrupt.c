/*
 * 002ButtonPressed.c
 *
 *  Created on: Feb 25, 2024
 *      Author: Rakesh Ahuja
 */

#include "stm32f407xx_gpio_driver.h"



void delay()
{
	for(uint32_t i = 0; i<500000; i++);
}
int main()
{
	GPIO_Handle_t gpioSwitch, gpioLed;
	memset(&gpioSwitch, 0, sizeof(gpioSwitch));
	memset(&gpioLed, 0, sizeof(gpioLed));

	//Switch
	gpioSwitch.pGPIOx = GPIOA;
	gpioSwitch.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioSwitch.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioSwitch.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioSwitch.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_PeriClock_Control(gpioSwitch.pGPIOx, ENABLE);
	GPIO_Init(&gpioSwitch);

	//LED
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_PeriClock_Control(gpioLed.pGPIOx, ENABLE);
	GPIO_Init(&gpioLed);

	//IRQ Configuration- First give priority and then enable it
	GPIO_IRQ_Priority_Config(EXTI0_IRQ_NUM, IRQ_PRIORITY_6);
	GPIO_IRQ_Config_ENorDI(EXTI0_IRQ_NUM, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	//call the IRQ handling driver
	delay();
	GPIO_IRQ_Handling(GPIO_PIN_NO_0);	//Calling to enable he EXTI Pending bit
	GPIO_Toggle(GPIOD, GPIO_PIN_NO_13);	//Toggling the LED
}
