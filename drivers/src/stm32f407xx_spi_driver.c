/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 2, 2024
 *      Author: Rakesh Ahuja
 */

#include "stm32f407xx_spi_driver.h"
//Helper function- Function that can be only be called by the source file of a driver and not the application
static void SPI_Close_Tx(SPI_Handle_t *pSPIHandle);
static void SPI_Close_Rx(SPI_Handle_t *pSPIHandle);

//Helper function
static void SPI_Close_Tx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
}

static void SPI_Close_Rx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
}
/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
	}
	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
	}
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);	//enable the clock
	uint16_t temp = 0;
	/*
	 * TODO 1 (SPI.c) - SPI_CR1 register
	 */
	//Device Mode
	temp = (pSPIHandle->spiConfig.SPI_DeviceMode << 2);

	//Bus Config
	if(pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUSCFG_FD)
	{
		//Bidi Mode clear
		temp &= ~(1 << 15);
	}
	else if(pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUSCFG_HD)
	{
		//Bidi Mode set
		temp |= (1 << 15);
	}
	else if(pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUSCFG_SIMPLE_RX)
	{
		//Bidi Cleared and Rx is set
		temp &= ~(1 << 15);
		temp |= (1 << 10);
	}

	//Speed
	temp |= pSPIHandle->spiConfig.SPI_Speed << 3;

	//Data Frame
	temp |= (pSPIHandle->spiConfig.SPI_DataFrame << 11);

	//Select slave management
	temp |= (pSPIHandle->spiConfig.SPI_SlaveSelect << SPI_CR1_SSM);

	//CPOL & CPHA
	temp |= pSPIHandle->spiConfig.SPI_CPOLValue << 1;
	temp |= pSPIHandle->spiConfig.SPI_CPHAValue << 0;

	pSPIHandle->pSPIx->CR1 &= ~(0xffff << 0);
	pSPIHandle->pSPIx->CR1 = temp;

}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else if(EnOrDi == DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else if(EnorDi == DISABLE)
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}


void SPI_Config_SSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}
	else if(EnorDi == DISABLE)
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}

/*
 * Name: SPI_SendData
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note: Blocking API
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen)
{
	while(dataLen)
	{
		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bit
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			dataLen -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
//			8bit
			pSPIx->DR = *pTxBuffer;
			dataLen--;
			pTxBuffer++;
		}
	}
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen)
{
	while(dataLen)
	{
		while(!(pSPIx->SR & ( 1 << SPI_SR_RXNE)));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			dataLen -= 2;
			(uint16_t *)pRxBuffer++;
		}
		else{
			*pRxBuffer = pSPIx->DR;
			dataLen--;
			pRxBuffer++;
		}
	}
}

void SPI_SendData_it(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen)
{
	//0) First check if the state is ready. If so only then continue
	if(pSPIHandle->TxState == SPI_BUSY_IN_TX)
	{
		//1) Save the TX buffer and length to global variable to be used later.
		//2) Store the state of SPI as Busy in TX
		pSPIHandle->TxLen = dataLen;
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3) Enable the TXEIE control bit to get the interrupt whenever the TXE flag is set
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}

}

void SPI_ReceiveData_it(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t dataLen)
{
	if(pSPIHandle->RxState == SPI_BUSY_IN_RX)
	{
		pSPIHandle->RxLen = dataLen;
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
	}
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_IRQinterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
	}
	else if(EnorDi == DISABLE)
	{
		if(IRQNumber <= 31)
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ICER1 &= ~(1 << IRQNumber % 32);
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ICER2 &= ~(1 << IRQNumber % 64);
	}
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_InterruptPriority(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t ip_reg = IRQNumber / 4, ip_start_bit = IRQNumber % 4;

	*(NVIC_PR + (ip_reg * 4)) |= (IRQPriority << ((8 * ip_start_bit) + (8 - NO_OF_IMPLEMENTED_BITS)));
}

/*
 * Name: SPI_PeriClockControl
 * Brief:
 * Param[1]:
 *
 * Return:
 * Note:
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	//Check for the TXE flag
	if((pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE)) && (pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE)))
	{
		while(pSPIHandle->TxLen)
		{
			if(pSPIHandle->spiConfig.SPI_DataFrame == SPI_DF_8BIT)
			{
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			else if(pSPIHandle->spiConfig.SPI_DataFrame == SPI_DF_16BIT)
			{
				pSPIHandle->pSPIx->DR = *((uint16_t *)(pSPIHandle->pTxBuffer));
				pSPIHandle->TxLen -= 2;
				(uint16_t *)pSPIHandle->pTxBuffer++;
			}
		}
		SPI_Close_Tx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
	//Check for the RXNE flag
	else if((pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE)) && (pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE)))
	{
		while(pSPIHandle->RxLen)
		{
			if(pSPIHandle->spiConfig.SPI_DataFrame == SPI_DF_8BIT)
			{
				*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
			}
			else if(pSPIHandle->spiConfig.SPI_DataFrame == SPI_DF_16BIT)
			{
				*((uint16_t *)(pSPIHandle->pRxBuffer)) = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen -= 2;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
		}

		//Deactivate the RXNIE, reduce the length and application callback
		SPI_Close_Rx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);


	}
	else if((pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR)) && pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE))
	{
		uint8_t temp;
		//Clear the OVR flag
		if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
		{
			//Clear the ovr bit by reading the DR register followed by reading the SR register
			temp = pSPIHandle->pSPIx->DR;
			temp = pSPIHandle->pSPIx->SR;
		}
		(void)temp;
		SPI_Close_Tx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_CMPLT);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv)
{
	//This is a week implementation. It depends on the user to provide a proper implememtation
}


