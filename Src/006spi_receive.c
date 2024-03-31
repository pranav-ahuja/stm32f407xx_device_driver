/*
 * 006spi_receive.c
 *
 *  Created on: Mar 7, 2024
 *      Author: Rakesh Ahuja
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#define NACK 0xA5
#define ACK 0xF5

//Arduino Pins
#define LED_PIN_ARD					9

//command codes
#define COMMAND_LED_CTRL          	0x50
#define COMMAND_SENSOR_READ       	0x51
#define COMMAND_LED_READ          	0x52
#define COMMAND_PRINT           	0x53
#define COMMAND_ID_READ         	0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

/*
 * Full Duplex
 * Master
 * DFF= 0
 * HW slave mgmt
 * Clock speed = 2MHz out of 16MHz
 *
 * Port A
 * Pin 4 - NSS
 * Pin 5 - SCK
 * Pin 6 - MISO
 * Pin 7 - MOSI
 */

void gpio_inits();
void spi_inits();
void btn_init();

void delay()
{
	for(int i = 0; i<5000000/2; i++);
}

void gpio_inits()
{
	GPIO_Handle_t gpioHandle;

	gpioHandle.pGPIOx = GPIOA;
	gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpioHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	gpioHandle.GPIO_PinConfig.GPIO_PinNumber = 4;	//NSS
	GPIO_Init(&gpioHandle);

	gpioHandle.GPIO_PinConfig.GPIO_PinNumber = 5;	//SCK
	GPIO_Init(&gpioHandle);

	gpioHandle.GPIO_PinConfig.GPIO_PinNumber = 6;	//MISO
	GPIO_Init(&gpioHandle);

	gpioHandle.GPIO_PinConfig.GPIO_PinNumber = 7;	//MOSI
	GPIO_Init(&gpioHandle);
}

void spi_inits()
{
	SPI_Handle_t spiHandle;

	spiHandle.pSPIx = SPI1;
	spiHandle.spiConfig.SPI_BusConfig = SPI_BUSCFG_FD;
	spiHandle.spiConfig.SPI_CPHAValue = 0;
	spiHandle.spiConfig.SPI_CPOLValue = 0;
	spiHandle.spiConfig.SPI_DataFrame = SPI_DF_8BIT;
	spiHandle.spiConfig.SPI_SlaveSelect = SPI_SSM_HW;
	spiHandle.spiConfig.SPI_Speed = SPI_SCLK_SPEED_DIV8;

	SPI_Init(&spiHandle);
}

void btn_init()
{
	GPIO_Handle_t btnHandle;

	btnHandle.pGPIOx = GPIOA;
	btnHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	btnHandle.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	btnHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	btnHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	btnHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&btnHandle);
}

int SPI_VerifyResponse(uint8_t ackornack)
{
	if(ackornack == ACK)
	{
		return 1;
	}
	else{
		return 0;
	}
}

int main()
{
	uint8_t rcvd_data, command, dummy = 0xff, pin;
	uint8_t ackByte;
	uint8_t args[2];
	uint16_t slave_id;

	gpio_inits();

	spi_inits();

	btn_init();

	SPI_Config_SSOE(SPI1, ENABLE);

	while(1)
	{
		/********************COMMAND LED CONTROL**************************/
		while(!(GPIO_ReadFromPin(GPIOA, GPIO_PIN_NO_0) == SET));

		delay();

		SPI_PeripheralControl(SPI1, ENABLE);

		command = COMMAND_LED_CTRL;

		SPI_SendData(SPI1, &command, 1);	//Sending command and wait for ack and nack

		//Reading the data stored in RXNE to clear the DR register which was filled during the transmission.
		SPI_ReceiveData(SPI1, &dummy, 1);

		//Send some dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI1, &dummy, 1);
		SPI_ReceiveData(SPI1, &ackByte, 1);	//Receiving the ack bit and wait

		while(SPI1->SR & (1 << SPI_SR_BSY));

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN_ARD;
			if(args[1] == LED_ON)
				args[1] = LED_OFF;
			else
				args[1] = LED_ON;

			SPI_SendData(SPI1, args, 2);
			while(SPI1->SR & (1 << SPI_SR_BSY));
		}


		/***********************COMMAND SENSOR READ*************************/
		while(!(GPIO_ReadFromPin(GPIOA, GPIO_PIN_NO_0) == SET));
		delay();
		rcvd_data = 0;
		command = COMMAND_SENSOR_READ;

		SPI_SendData(SPI1, &command, 1);	//Sending command
		SPI_ReceiveData(SPI1, &dummy, 1);	//Reading the dummy data

		SPI_SendData(SPI1, &dummy, 1);		//Send the dummy data to receive the data from the slave
		SPI_ReceiveData(SPI1, &ackByte, 1);	//Receive an acknowledge bit from the slave

		if(SPI_VerifyResponse(ackByte))		//Verify the ack or nack bit
		{
			pin = ANALOG_PIN0;

			SPI_SendData(SPI1, &pin, 1);	//Send the pin number
		}
		SPI_ReceiveData(SPI1, &dummy, 1);	//Dummy read to clear RXNE
		delay();

		SPI_SendData(SPI1, &dummy, 1);		//Dummy read
		SPI_ReceiveData(SPI1, &rcvd_data, 1);	//Read the sensor value



		/*******************COMMAND LED READ*********************************/
		while(!(GPIO_ReadFromPin(GPIOA, GPIO_PIN_NO_0) == SET));
		delay();
		rcvd_data = 0;
		command = COMMAND_LED_READ;
		pin = LED_PIN_ARD;

		SPI_SendData(SPI1, &command, 1);		//Command to read the LED value
		SPI_ReceiveData(SPI1, &dummy, 1);		//Reading the dummy data to clear the RXNE

		SPI_SendData(SPI1, &dummy, 1);			//Sending the dummy data to receive the ack byte
		SPI_ReceiveData(SPI1, &ackByte, 1);		//Receiving the acknowledge bit

		if(SPI_VerifyResponse(ackByte))
		{
			SPI_SendData(SPI1, &pin, 1);		//Send the pin number
		}
		SPI_ReceiveData(SPI1, &dummy, 1);		//Reading the received data to clear the RXNE bit
		delay();

		SPI_SendData(SPI1, &dummy, 1);			//Sending the dummy data to receive the value of the pin
		SPI_ReceiveData(SPI1, &rcvd_data, 1);	//Read the value of the pin

		/*******************COMMAND PRINT*********************************/
		while(!(GPIO_ReadFromPin(GPIOA, GPIO_PIN_NO_0) == SET));
		delay();
		command = COMMAND_PRINT;
		char message[] = "Hi";

		SPI_SendData(SPI1, &command, 1);		//Send the command to the slave
		SPI_ReceiveData(SPI1, &dummy, 1);		//Reading the received data to clear the RXNE

		SPI_SendData(SPI1, &dummy, 1);			//Sending the dummy value to read the ack bit
		SPI_ReceiveData(SPI1, &ackByte, 1);		//Read the acknowledge byte

		if(SPI_VerifyResponse(ackByte))
		{
			SPI_SendData(SPI1, (uint8_t*)message, strlen(message));  	//Send the message to the slave
		}
		SPI_ReceiveData(SPI1, &dummy, 1);		//To read the received data to clear the RXNE


		/*******************CMD_ID_READ*********************************/
		while(!(GPIO_ReadFromPin(GPIOA, GPIO_PIN_NO_0) == SET))
		{
			delay();
			command = COMMAND_ID_READ;

			SPI_SendData(SPI1, &command, 1);	//Send the commad to the slave
			SPI_ReceiveData(SPI1, &dummy, 1);	//Read the received data to clear the RXNE

			SPI_SendData(SPI1, &dummy, 1);		//Send the dummy data to receive the ack
			SPI_ReceiveData(SPI1, &ackByte, 1);	//Receive the ack bit

			if(SPI_VerifyResponse(ackByte))
			{
				SPI_SendData(SPI1, &dummy, 10);	//Send the dummy data to receive the slave id
				SPI_ReceiveData(SPI1, (uint8_t *)(&slave_id), 1);	//Receive the slave ID
			}
		}
	}


	return 0;
}
