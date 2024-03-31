/*
 * 004SPITestTx.c
 *
 *  Created on: Mar 4, 2024
 *      Author: Rakesh Ahuja
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

void spiGpio_Inits(void)
{
	//PB15- MOSI
	//PB14 - MISO
	//PB13 = SCLK
	//PB12 = NSS
	//Alt Func Mode = 5

	GPIO_Handle_t spiPins;
	memset(&spiPins, 0, sizeof(spiPins));

	spiPins.pGPIOx = GPIOB;
	spiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	spiPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	spiPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;	//SCLK - ch1
	GPIO_Init(&spiPins);

	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;	//MOSI - ch-2
	GPIO_Init(&spiPins);

	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;	//MISO - ch-3
	GPIO_Init(&spiPins);

	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;	//NSS - ch4 - ch4
	GPIO_Init(&spiPins);

}

int main(void)
{
	char userData[] = "Hello World";
	SPI_Handle_t spiHandle;

	memset(&spiHandle, 0, sizeof(spiHandle));

	spiGpio_Inits();

	spiHandle.pSPIx = SPI2;
	spiHandle.spiConfig.SPI_DeviceMode = SPI_MASTER_MODE;
	spiHandle.spiConfig.SPI_Speed = SPI_SCLK_SPEED_DIV2;
	spiHandle.spiConfig.SPI_DataFrame = SPI_DF_8BIT;
	spiHandle.spiConfig.SPI_BusConfig = SPI_BUSCFG_FD;
	spiHandle.spiConfig.SPI_CPOLValue = 0;
	spiHandle.spiConfig.SPI_CPHAValue = 0;
	spiHandle.spiConfig.SPI_SlaveSelect = SPI_SSM_SW;

	SPI_Init(&spiHandle);
	SPI_SSI_Config(spiHandle.pSPIx, ENABLE);	//enabling SSI to prevent the MODF fault and then enable the SPI
	SPI_PeripheralControl(spiHandle.pSPIx, ENABLE);	//Enable the SPI peripheral

	SPI_SendData(spiHandle.pSPIx, (uint8_t*)userData, strlen(userData));

	SPI_SSI_Config(spiHandle.pSPIx, DISABLE);

	while(1);
	return 0;
}
