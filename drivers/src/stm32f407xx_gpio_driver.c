/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Rakesh Ahuja
 */


#include "stm32f407xx_gpio_driver.h"

/*
 * TODO 1 - GPIO.c:
 * @Func - GPIO_Init
 * @Brief - This will initialize the gpio peripheral
 * @para[1] - pGPIOHandle -> To point to both the config structure and the GPIO structure
 * @Return - None
 * @Note -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) 	//Initializes the GPIO port
{
	GPIO_PeriClock_Control(pGPIOHandle->pGPIOx, ENABLE);	//Enable the clock

	uint32_t temp = 0;
	//TODO 1.1 - GPIO.c - Configure Modes
	//If the GPIO Mode value is less than Analog Input, it will behave as normal mode. If it is greater then it will behave as Interrupt
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Normal Mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//Clearing
		pGPIOHandle->pGPIOx->MODER |= temp;	//Setting
	}
	else
	{
		//TODO 1.2 - GPIO.c - Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Configure falling edge trigger (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Set the EXTI bit for the Pin Number
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clear the EXTI bit for the Pin Number

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Configure rising edge trigger by setting RTSR and clearing FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure rising and falling edge trigger by setting the corresponding bit in RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//TODO 1.3 - GPIO.c - Configure the SYSCFG EXTI registerS

		SYSCFG_PCLK_EN();		//Enable the clock

		//Declare variable to store the value of the GPIO Port and get the value for a particular GPIO
		uint8_t exti_val_gpio = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		//Write the value to the SYSCFG EXTI Control register
		SYSCFG->EXTICR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4] = (exti_val_gpio << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4));

		//TODO 1.4 - GPIO.c - Enable the external interrupt using IMR register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//TODO 1.2 - GPIO.c - Configure speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//TODO 1.3 - GPIO.c - Configure pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//TODO 1.4 - GPIO.c - Configure output type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//TODO 1.5 - GPIO.c - Configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t afr_num = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t af_start = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * af_start);
		pGPIOHandle->pGPIOx->AFR[afr_num] &= ~(15 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFR[afr_num] |= temp;
		temp = 0;
	}

}

/*
 * TODO 2 - GPIO.c:
 * @Func - GPIO_DeInit
 * @Brief - This is used by the user to de-initialize the GPIO peripherals
 * @para[1] - *pGPIOx - Used by user to select the GPIO Port
 * @Return - None
 * @Note -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)					//De-initializes the GPIO Port
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOA_REG_RESET();
}

/*
 * TODO 3 - GPIO.c:
 * @Func - GPIO_PeriClock_Control
 * @Brief - To enable and disable the peripheral clock of GPIO based on user input
 * @para[1] - Asking user to enter the port
 * @para[2] - Asking user to enter ENABLE or DISABLE
 * @Return - None
 * @Note - None
 */
void GPIO_PeriClock_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)				//Enable and Disable the clock
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else if(EnOrDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
	}
}

/*
 * TODO 4 - GPIO.c:
 * @Func - GPIO_ReadFromPin
 * @Brief - To read the data from the pin
 * @para[1] - pGPIOx -> To get the GPIO Port from the user
 * @para[2] - PinNumber -> To get the pin number from the user
 * @Return - 8bit value - uint8_t
 * @Note -
 */
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)			//Read from the Pin
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/*
 * TODO 5 - GPIO.c:
 * @Func - GPIO_ReadFromPort
 * @Brief - To read input from the GPIO Port
 * @para[1] - *pGPIOx -> This will collect the port provided by the user
 * @Return - uint16_t value of input register
 * @Note -
 */
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)		//Read from the Port. Port is of 16 pins(16 bits)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/*
 * TODO 6 - GPIO.c:
 * @Func - GPIO_WriteToPin
 * @Brief - To provide output from the gpio port pin
 * @para[1] - *pGPIOx -> To collect the GPIO Port from the user
 * @para[2] - PinNumber -> To collect the pin of a GPIO port provided by the user
 * @para[3] - SetOrReset -> To set or reset the pin of the GPIO port
 * @Return - none
 * @Note -
 */
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t SetOrReset_val)				//Write to the pin
{
	if(SetOrReset_val == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else if(SetOrReset_val == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * TODO 7 - GPIO.c:
 * @Func - GPIO_WriteToPort
 * @Brief - To provide output from the GPIO port pin
 * @para[1] - *pGPIOx -> To collect the GPIO Port from the user
 * @para[2] - data - > To collect the 16 bit data from the user
 * @Return - none
 * @Note -
 */
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t data)			//Write to the port. 16 pins in a port(16 bit)
{
	pGPIOx->ODR = data;
}

/*
 * TODO 8 - GPIO.c:
 * @Func - GPIO_Init
 * @Brief -
 * @para[1] -
 * @Return -
 * @Note - Processor side configuration
 */
void GPIO_IRQ_Config_ENorDI(uint8_t IRQNumber, uint8_t EnOrDi)				//Configs of an IRQ
{
	if(EnOrDi == ENABLE)
	{
		//Configure the interrupt set enable register
		if(IRQNumber <=31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	}
	else if(EnOrDi == DISABLE)
	{
		//Configure the interrupt clear enable register
		if(IRQNumber <=31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 * TODO 9 - GPIO.c:
 * @Func - GPIO_Init
 * @Brief -
 * @para[1] -
 * @Return -
 * @Note -
 */
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;	//Get the register for the IRQ
	uint8_t ipr_start = IRQNumber % 4;	//Get the starting bit of the IRQ priority

	*(NVIC_PR + (iprx*4)) = (IRQPriority << ((ipr_start * 8) + NO_OF_IMPLEMENTED_BITS)) ;	//Put the priority value in the register
}

/*
 * TODO 10 - GPIO.c:
 * @Func - GPIO_Init
 * @Brief -
 * @para[1] -
 * @Return -
 * @Note -
 */
void GPIO_IRQ_Handling(uint8_t PinNumber)			//Handling of an IRQ
{
	//1. Implement ISR handling
	//Check if the internal interrupt occur? If so then clear the pending register.
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);	//This is cleared by writing 1 to the bit
	}

	//2. Store the address of ISR at the vector table location corresponding to the IRQ number for which the ISR has been written - Will be taken care by the startup file.
}

/*
 * TODO 10 - GPIO.c:
 * @Func - GPIO_Toggle
 * @Brief - To toggle the GPIO pin
 * @para[1] - pGPIOx -> To collect the Port from the user
 * @para[2] - PinNumber -> To collect the pin number from the user
 * @Return - none
 * @Note -
 */
void GPIO_Toggle(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)					//testing
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
