/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Rakesh Ahuja
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * ToDo GPIO-1: Configurable Structure - To collect the configuration done by the user
 */
typedef struct{
	uint8_t GPIO_PinNumber;	//Selects the pin number - @GPIO_PIN_NO
	uint8_t GPIO_PinMode;	//select the pin mode, output or input - @GPIO_MODE
	uint8_t GPIO_PinSpeed;	//Speed of the pin	- @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;	//Specifying push or pull	-	@GPIO_NO_PUPD
	uint8_t GPIO_PinOPType;	//Select the output type	- @GPIO_OPTYPE
	uint8_t GPIO_PinAltFunMode;	//Select the alternate function mode
}GPIO_PinConfig_t;

/*
 * ToDo GPIO-2: Handler Structure - To Collect all the configurable data and the GPIO port and send it to the API
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;	//Hold the base address of the gpio port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * ToDo GPIO-3: Macros for GPIO Peripherals
 */
//Modes
#define GPIO_MODE_INPUT					0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALTFN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_RFT				6


//Output Types
#define GPIO_OPTYPE_PP					0
#define GPIO_OPTYPE_OD					1


//Speed
#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MEDIUM				1
#define GPIO_SPEED_FAST					2
#define GPIO_SPEED_HIGH					3


//Pull up and Pull down config
#define GPIO_NO_PUPD					0
#define GPIO_PIN_PU						1
#define GPIO_PIN_PD						2


//Pin Numbers
#define GPIO_PIN_NO_0					0
#define GPIO_PIN_NO_1					1
#define GPIO_PIN_NO_2					2
#define GPIO_PIN_NO_3					3
#define GPIO_PIN_NO_4					4
#define GPIO_PIN_NO_5					5
#define GPIO_PIN_NO_6					6
#define GPIO_PIN_NO_7					7
#define GPIO_PIN_NO_8					8
#define GPIO_PIN_NO_9					9
#define GPIO_PIN_NO_10					10
#define GPIO_PIN_NO_11					11
#define GPIO_PIN_NO_12					12
#define GPIO_PIN_NO_13					13
#define GPIO_PIN_NO_14					14
#define GPIO_PIN_NO_15					15


/*
 * ToDo GPIO-100: APIs to the user
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);											//Initializes the GPIO port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);											//Deinitializes the GPIO Port
void GPIO_PeriClock_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);					//Enable and Disable the clock
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);					//Read from the Pin
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);									//Read from the Port. Port is of 16 pins(16 bits)
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t SetOrReset);	//Write to the pin
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t data);						//Write to the port. 16 pins in a port(16 bit)
void GPIO_IRQ_Config_ENorDI(uint8_t IRQNumber, uint8_t EnOrDi);						//Configs of an IRQ
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQ_Handling(uint8_t PinNumber);											//Handling of an IRQ


void GPIO_Toggle(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);							//testing


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
