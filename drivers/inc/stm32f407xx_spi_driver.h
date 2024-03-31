/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 2, 2024
 *      Author: Rakesh Ahuja
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * TODO 1(SPI.h) - Structures for SPI Configuration and handling
 */
typedef struct{
	uint8_t SPI_DeviceMode;		//Master or slave
	uint8_t SPI_BusConfig;		//Full duplex or half duplex or simplex
	uint8_t SPI_Speed;			//Speed of the SPI
	uint8_t SPI_DataFrame;		//8 bit or 16bit of data and shift register will shift
	uint8_t SPI_CPHAValue;		//1 or 0
	uint8_t SPI_CPOLValue;		//1 or 0
	uint8_t SPI_SlaveSelect;	//Software or Hardware slave management
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;	//Store the base address of the SPI peripheral selected
	SPI_Config_t spiConfig;	//Store the values of the SPI configurations done on the SPI protocol
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


/*
 * TODO 2(SPI.h) - Macros for the SPI Configuration and more
 */

//SPI Mode
#define SPI_MASTER_MODE				1
#define SPI_SLAVE_MODE				0

//SPI Bus Config
#define SPI_BUSCFG_FD				0
#define SPI_BUSCFG_HD				1
#define SPI_BUSCFG_SIMPLE_RX		2

//Clock Speed
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

//Data Frame
#define SPI_DF_8BIT					0
#define SPI_DF_16BIT				1

//CPOL Value
#define SPI_CPOL_0					0
#define SPI_CPOL_1					1

//CPHA
#define SPI_CPHA_0					0
#define SPI_CPHA_1					1

//SPI Enable and disable
#define SPI_DISABLE					0
#define SPI_ENABLE					1

//Data Frame Format
#define SPI_MSB_TX_FIRST			0
#define SPI_LSB_TX_FIRST			1

//Slave management
#define SPI_SSM_HW					0
#define SPI_SSM_SW					1

//SPI State
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

//Possible SPI Events
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_CMPLT			3
/*
 * TODO 3(SPI.h) - Function declarations/prototype
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Config_SSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen);

void SPI_SendData_it(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen);
void SPI_ReceiveData_it(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t dataLen);

void SPI_IRQinterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_InterruptPriority(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

//Application call back to the application so that the user can decide what to do if the interrupt is triggered
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
