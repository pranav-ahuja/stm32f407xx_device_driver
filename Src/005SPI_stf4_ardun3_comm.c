/*
 * SPI_stf4_ardun3_comm.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Rakesh Ahuja
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

void Gpios_Inits();
void SPI_Pins_Init();
void GPIOs_Btn_Init();

void delay()
{
	for(int i = 0; i<500000/2; i++);
}

void Gpios_Inits()
{
	GPIO_Handle_t gpio_spipins;
	memset(&gpio_spipins, 0, sizeof(gpio_spipins));

	gpio_spipins.pGPIOx = GPIOA;
	gpio_spipins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_spipins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	gpio_spipins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpio_spipins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_spipins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;		//For SPI1

	gpio_spipins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;	//NSS
	GPIO_Init(&gpio_spipins);

	gpio_spipins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;	//SCLK
	GPIO_Init(&gpio_spipins);

//	gpio_spipins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;	//MISO
//	GPIO_Init(&gpio_spipins);

	gpio_spipins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;	//MOSI
	GPIO_Init(&gpio_spipins);
}

void SPI_Pins_Init()
{
	SPI_Handle_t spi_pins;

	spi_pins.pSPIx = SPI1;
	spi_pins.spiConfig.SPI_DeviceMode = SPI_MASTER_MODE;
	spi_pins.spiConfig.SPI_BusConfig = SPI_BUSCFG_FD;
	spi_pins.spiConfig.SPI_DataFrame = SPI_DF_8BIT;
	spi_pins.spiConfig.SPI_Speed = SPI_SCLK_SPEED_DIV8;
	spi_pins.spiConfig.SPI_CPHAValue = 0;
	spi_pins.spiConfig.SPI_CPOLValue = 0;
	spi_pins.spiConfig.SPI_SlaveSelect = SPI_SSM_HW;

	SPI_Init(&spi_pins);
}

void GPIOs_Btn_Init()
{
	GPIO_Handle_t btnhandle;

	btnhandle.pGPIOx = GPIOA;
	btnhandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	btnhandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
//	btnhandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	btnhandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	btnhandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&btnhandle);
}

int main()
{
	char data[]= "Hi, Hello, Bye";
	uint8_t length = strlen(data);
	//GPIOs
	Gpios_Inits();

	//Master SPI
	SPI_Pins_Init();

	//Btn Init
	GPIOs_Btn_Init();

	//SSOE
	SPI_Config_SSOE(SPI1, ENABLE);	//Enable SSOE to make the NSS pin 0 when SPE is enabled and 1 when spe is disabled

	while(1)
	{
		while(!(GPIO_ReadFromPin(GPIOA, 0) == SET));

		delay();
		SPI_PeripheralControl(SPI1, ENABLE);

		//Send data
		//first send number of bytes
		SPI_SendData(SPI1, &length, sizeof(length));
		//then send the data
		SPI_SendData(SPI1, (uint8_t*)data, strlen(data));

		while(SPI1->SR & (1 << SPI_SR_BSY));

		SPI_PeripheralControl(SPI1, DISABLE);


	}
//	SPI_SSI_Config(SPI1, ENABLE);
//	SPI_SSI_Config(SPI1, DISABLE);

	return 0;
}
