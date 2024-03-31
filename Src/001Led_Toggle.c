/*
 * 001Led_Toggle.c
 *
 *  Created on: Feb 25, 2024
 *      Author: Rakesh Ahuja
 */

#include "stm32f407xx_gpio_driver.h"

//Delay function
void delay()
{
	for(uint32_t i = 0; i<500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOD;											//Set the GPIO Port as GPIOD
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;			//Set the Pin Number as 12
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;			//Select the mode as the output

	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;			//Select output type as Push and Pull

	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;			//Select speed of output as fast
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;		//Select the no pull-up or pull down configuration

	GPIO_PeriClock_Control(gpioLed.pGPIOx, ENABLE);					//Enable the GPIOD  peripheral clock

	GPIO_Init(&gpioLed);											//initialize the GPIO with all the above mentioned configurations

	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;			//Now select pin 13 or GPIO D port and initialize the GPIOD pin 13
	GPIO_Init(&gpioLed);

	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;			//Now select pin 14 or GPIO D port and initialize the GPIOD pin 14
	GPIO_Init(&gpioLed);

	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;			//Now select pin 15 or GPIO D port and configure the GPIOD pin 15
	GPIO_Init(&gpioLed);

	//Run the while loop to toggle the LED again and again
	while(1)
	{
		//Toggle LEDs with all the below mentioned pins
		GPIO_Toggle(gpioLed.pGPIOx, GPIO_PIN_NO_12);
		GPIO_Toggle(gpioLed.pGPIOx, GPIO_PIN_NO_13);
		GPIO_Toggle(gpioLed.pGPIOx, GPIO_PIN_NO_14);
		GPIO_Toggle(gpioLed.pGPIOx, GPIO_PIN_NO_15);

		//Introduce a delay
		delay();
	}
}
