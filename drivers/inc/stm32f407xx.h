/*
 * STM32F407XX.h
 *
 *  Created on: Feb 20, 2024
 *      Author: Pranav Ahuja
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>
#include<string.h>

#define __weak 				__attribute__((weak))

/*
 * TODO -1: PROCESSOR SPECIFIC DETAILS
 */
//NVIC Registers
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3			((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2			((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)

#define NVIC_ISPR0			((volatile uint32_t*)0XE000E200)
#define NVIC_ISPR1			((volatile uint32_t*)0XE000E204)
#define NVIC_ISPR2			((volatile uint32_t*)0XE000E208)
#define NVIC_ISPR3			((volatile uint32_t*)0XE000E20C)

#define NVIC_PR				((volatile uint32_t*)0xE000E400)

#define NO_OF_IMPLEMENTED_BITS	4

/*
 * TODO 1: Define the base addresses of various clocks, memories and peripherals of the respective buses
 */

/*
 * base addresses of flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U
#define OTP_AREA_BASEADDR	0x1FFF7800U	//One Time programmable
#define SRAM 				SRAM1_BASEADDR

/*
 * Bus Addresses of System bus
 */
#define PER_BASEADDRESS		0x40000000U
#define APB1_BASEADDR		PER_BASEADDRESS
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U

/*
 * Base Address of AHB1 peripheral bus
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1_BASEADDR + 0x2000)
#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)

/*
 * Base Addresses of APB1 Bus
 */
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00)
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0X5000)

/*
 * Base Address of APB2 Bus
 */
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)

/*
 * ***************************************Peripheral Register Defintion Structures***************************************
 */

/*
 * TODO 2:Peripheral Register Definition Structures
 */

/*
 * TODO 2.1: Structure for GPIOs
 */
typedef struct
{
	volatile uint32_t MODER;	//mode register		- 00
	volatile uint32_t OTYPER;	//output type register	- 04
	volatile uint32_t OSPEEDR;	//output speed register - 08
	volatile uint32_t PUPDR;	//push pull register -
	volatile uint32_t IDR;		//input data register
	volatile uint32_t ODR;		//output data register
	volatile uint32_t BSRR;		//bit set/reset register
	volatile uint32_t LCKR;		//Lock register
	volatile uint32_t AFR[2];	//alternate function register. AFR[0] - low ; AFR[1] - high
}GPIO_RegDef_t;

/*
 * TODO 2.2: RCC Peripheral Register Definition Structure
 */
typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t 		  Reserved0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t   		  Reserved1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t 		  Reserved2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t 		  Reserved3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t 		  Reserved4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t 		  Reserved5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t 		  Reserved6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;


/*
 * TODO 2.3: EXTI & Syscfg Peripheral structure
 */

typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t Reserved1[2];
	volatile uint32_t CMPCR;
}SYSCFG_RegDeg_t;

/*
 * TODO 2.4: SPI Peripheral structure
 */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * TODO 2.5: I2C Peripheral structure
 */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

/*
 * TODO 3 - Peripheral Macros
 * TODO 3.1 - Macros for GPIO peripherals
 */
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)


 //TODO 3.2 - Macros for RCC Peripheral Structure
#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)


//TODO 3.3 - Macros for EXTI Peripheral
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)


// TODO 3.4 - Macros for SYSCFG peripherals
 #define SYSCFG				((SYSCFG_RegDeg_t*)SYSCFG_BASEADDR)

//TODO 3.5 - Macros for SPI Peripheral base address
#define SPI1				((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t *)SPI3_BASEADDR)

//TODO 3.6 - Macros for I2C Peripherals
#define I2C1				((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t *)I2C3_BASEADDR)

/*
 * TODO 4: Clock Enable Macros for GPIOx Peripherals
 */
//Enable
#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()			( RCC->AHB1ENR |= (1<<8) )

//Disable
#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~(1<<8) )


//MISC: Resetting the GPIO peripherals

#define GPIOA_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * TODO 5: Clock Enable and Disable Macros for I2C Peripherals
 */
//Enable
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1<<23))

//Disable
#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1<<23))

//Register
//CR1
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NO_STRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

//CR2

//SR1
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				8
#define I2C_SR1_BERR			9
#define I2C_SR1_ARLO			10
#define I2C_SR1_AF				11
#define I2C_SR1_OVR				12
#define I2C_SR1_PECERR			14
#define I2C_SR1_TIMEOUT			15
#define I2C_SR1_SMBLERT			15


//SR2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GEN_CALL		4
#define I2C_SR2_SMBDE_FAULT		5
#define I2C_SR2_SMB_HOST		6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

//CCR
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_F_S				15

//OAR1
#define I2C_OAR1_ADD_0			0
#define I2C_OAR1_ADD_7_1		1
#define I2C_OAR1_ADD_9_8		8
#define I2C_OAR1_ADDMODE		15


/*
 * TODO 6: Clock Enable and Disable Macros for SPI Peripherals
 */
//Enable
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1<<15))

//Disable
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1<<15))

//Resetting the peripheral
#define SPI1_REG_RESET()		( RCC->APB2RSTR |= (1<<12))
#define SPI2_REG_RESET()		( RCC->APB1RSTR |= (1<<14))
#define SPI3_REG_RESET()		( RCC->APB1RSTR |= (1<<15))

//Maintaining Bit macros for SPI
//CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

//CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

//SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

//IRQ numbers for SPI
#define SPI1_IRQ_NUM			35
#define SPI2_IRQ_NUM			36
#define SPI3_IRQ_NUM			51


/*
 * TODO 7: Clock enable and disable macros for USART Peripherals
 */
//Enable
#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1<<18))

//Disable
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1<<18))

/*
 * TODO 8: Clock enable and disable macros for UART Peripherals
 */
//Enable
#define UART4_PCLK_EN()			( RCC->APB2ENR |= (1<<19))
#define UART5_PCLK_EN()			( RCC->APB2ENR |= (1<<20))

//Disable
#define UART4_PCLK_DI()			( RCC->APB2ENR &= ~(1<<19))
#define UART5_PCLK_DI()			( RCC->APB2ENR &= ~(1<<20))

/*
 * TODO 9: Clock enable and disable macros for SYSConfig Peripherals
 */
//Enable
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1<<14))

//Disable
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1<<14))


/*
 * TODO 10: SYSCFG values for GPIO Ports
 */
#define SYSCFG_EXTI_PORTA		0
#define SYSCFG_EXTI_PORTB		1
#define SYSCFG_EXTI_PORTC		2
#define SYSCFG_EXTI_PORTD		3
#define SYSCFG_EXTI_PORTE		4
#define SYSCFG_EXTI_PORTF		5
#define SYSCFG_EXTI_PORTG		6
#define SYSCFG_EXTI_PORTH		7

/*
 * TODO 11: Macros for interrupt mapping
 */
#define EXTI0_IRQ_NUM			6
#define EXTI1_IRQ_NUM			7
#define EXTI2_IRQ_NUM			8
#define EXTI3_IRQ_NUM			9
#define EXTI4_IRQ_NUM			10
#define EXTI5_9_IRQ_NUM			23
#define EXTI10_15_IRQ_NUM		40

/*
 * TODO 12: IRQ Priority
 */
#define IRQ_PRIORITY_0			0
#define IRQ_PRIORITY_1			1
#define IRQ_PRIORITY_2			2
#define IRQ_PRIORITY_3			3
#define IRQ_PRIORITY_4			4
#define IRQ_PRIORITY_5			5
#define IRQ_PRIORITY_6			6
#define IRQ_PRIORITY_7			7
#define IRQ_PRIORITY_8			8
#define IRQ_PRIORITY_9			9
#define IRQ_PRIORITY_10			10
#define IRQ_PRIORITY_11			11
#define IRQ_PRIORITY_12			12
#define IRQ_PRIORITY_13			13
#define IRQ_PRIORITY_14			14
#define IRQ_PRIORITY_15			15



/*
 * TODO 100: Macros for the user API
 */
//GPIO
#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

//Changing GPIO base address to CODE
#define GPIO_BASEADDR_TO_CODE(x)		( (x == GPIOA)?0 :\
										  (x == GPIOB)?1 :\
										  (x == GPIOC)?2 :\
										  (x == GPIOD)?3 :\
										  (x == GPIOE)?4 :\
										  (x == GPIOF)?5 :\
										  (x == GPIOG)?6 :\
										  (x == GPIOH)?7 :0)

#endif /* STM32F407XX_H_ */

