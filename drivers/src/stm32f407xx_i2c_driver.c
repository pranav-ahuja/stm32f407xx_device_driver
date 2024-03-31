/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Mar 20, 2024
 *      Author: Rakesh Ahuja
 */


#include "stm32f407xx_i2c_driver.h"

//Helper functions
static void I2C_Clock_Peripheral(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
static uint16_t calculateCCRVal(I2C_Handle_t *pI2C_Handle, uint32_t pClk);
static uint32_t RCC_GetPclk1Value(void);
static void I2C_WriteAddrAndReadNWrite(I2C_Handle_t *pI2C_Handle, uint8_t rnW, uint8_t slave_addr);

uint32_t AHB1Prescalers[] = {2,4,8,16,64,128,256, 512};
uint32_t APB1Prescalers[] = {2,4,8,16};

static uint32_t RCC_GetPclk1Value(void)
{
	uint32_t pClk, SystemClk, ahb1PreS;
	uint8_t temp, apb1PreS;

	//Check which clock source is being used. HSE, HSI, PLL
	temp = ((RCC->CFGR >> 2) & 3);

	//based on the clock source, update the systemclk variable with the value of each clock
	if(temp == 0)
		SystemClk = 16000000;
	else if(temp == 1)
		SystemClk = 8000000;
	else if(temp == 2)
		SystemClk = RCC_GetPLLClockOutput();

	//Check for the AHB prescalar vlaue and store it in temp variable.
	//Based on this temp, get the value of ahb1 prescalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
		ahb1PreS = 1;
	else if(temp >= 8)
		ahb1PreS = AHB1Prescalers[temp - 8];

	//Check for the APB prescalar value and store it in temp variable.
	//Based on this temp, get the value of apb1 prescalar
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
		apb1PreS = 1;
	else if(temp >= 4)
		apb1PreS = APB1Prescalers[temp - 4];

	pClk = (SystemClk / ahb1PreS) / apb1PreS;

	return pClk;
}

static uint16_t calculateCCRVal(I2C_Handle_t *pI2C_Handle, uint32_t pClk)
{
	uint32_t ccr_val = 0;
	//SM Mode
	if(pI2C_Handle->i2cConfig.I2C_Mode <= I2C_SM_MODE && pI2C_Handle->i2cConfig.I2C_SclSpeed <= I2C_SPEED_SM)
	{
		ccr_val = pClk / (2 * pI2C_Handle->i2cConfig.I2C_SclSpeed);
	}
	else if(pI2C_Handle->i2cConfig.I2C_Mode == I2C_FM_MODE)
	{
		if(pI2C_Handle->i2cConfig.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_2)
		{
			ccr_val = pClk / (3 * pI2C_Handle->i2cConfig.I2C_SclSpeed);
		}
		else if(pI2C_Handle->i2cConfig.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_16_9)
		{
			ccr_val = pClk / (25 * pI2C_Handle->i2cConfig.I2C_SclSpeed);
		}
	}

	return ccr_val;
}

static void I2C_Clock_Peripheral(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if(pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if(pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
}

static void I2C_WriteAddrAndReadNWrite(I2C_Handle_t *pI2C_Handle, uint8_t rnW, uint8_t slave_addr)
{
	if(pI2C_Handle->i2cConfig.I2C_AddressMode == I2C_7_BIT_ADD)
	{
		slave_addr = slave_addr << 1;	//Shifting slave address by 1 bit

		//Selecting read or write based on the command
		if(rnW == I2C_CMD_READ)
		{
			slave_addr |= (1 << 0);
		}
		else if(rnW == I2C_CMD_WRITE)
		{
			slave_addr &= ~(1 << 0);
		}

		//writing DR with the slave address with read/write bits
		pI2C_Handle->pI2Cx->DR |= (slave_addr);	//write the last bit
	}
	else if(pI2C_Handle->i2cConfig.I2C_AddressMode == I2C_10_BIT_ADD)
	{
		//create header
		uint8_t header = ((slave_addr >> 8) << 1) | I2C_HEADER_10_ADD;

		pI2C_Handle->pI2Cx->DR |= header;

		//ADD10 header check along with reading the SR1 to clear it
		while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADD10)));

		//Write address to the DR to clear the ADDR
		pI2C_Handle->pI2Cx->DR |= slave_addr;

	}

}

void I2C_Init(I2C_Handle_t *pI2C_Handle)
{
	uint8_t temp;
	uint32_t pClk=0;
	uint16_t ccr_val=0;
	uint32_t t_rise = 0;

	//1) Configure the modes
	temp = pI2C_Handle->i2cConfig.I2C_Mode;
	pI2C_Handle->pI2Cx->CCR |= (temp << I2C_CCR_F_S);
	temp = 0;

	//2) Configure Duty Cycle for FM
	if(pI2C_Handle->i2cConfig.I2C_Mode == I2C_FM_MODE)
	{
		temp = pI2C_Handle->i2cConfig.I2C_FMDutyCycle;
		pI2C_Handle->pI2Cx->CCR |= (temp << I2C_CCR_DUTY);
	}


	//3) Configure speed
	//3.1) FREQ Field
	pClk = RCC_GetPclk1Value() / 1000000U;	//The function will  provide us the value in MHz. So we have to divide it by 1MHz to get the value for FREQ
	pI2C_Handle->pI2Cx->CR2 |= (pClk & 0x3F);	//Mask all the bits except the first 5 bits

	//3.2) calculate the CCR value
	ccr_val = calculateCCRVal(pI2C_Handle, pClk);
	pI2C_Handle->pI2Cx->CCR |= ((ccr_val & 0xFFF) << I2C_CCR_CCR);


	//4) Device Address Mode
	if(pI2C_Handle->i2cConfig.I2C_AddressMode == I2C_10_BIT_ADD)
		pI2C_Handle->pI2Cx->OAR1 |= (1 << I2C_OAR1_ADDMODE);
	else if(pI2C_Handle->i2cConfig.I2C_AddressMode == I2C_7_BIT_ADD)
		pI2C_Handle->pI2Cx->OAR1 &= ~(1 << I2C_OAR1_ADDMODE);

	//5) Device Own Address;
	temp = pI2C_Handle->i2cConfig.I2C_DeviceAddress;
	if(pI2C_Handle->i2cConfig.I2C_AddressMode == I2C_10_BIT_ADD)
	{
		pI2C_Handle->pI2Cx->OAR1 |= (temp & (0x3FF));
	}
	else{
		pI2C_Handle->pI2Cx->OAR1 |= ((temp &(0x7F)) << I2C_OAR1_ADD_7_1);
	}

	pI2C_Handle->pI2Cx->OAR1 = (1 << 14);	//According to reference manual, this bit should be kept at 1


	//6) Configure Ack-ing
	temp = pI2C_Handle->i2cConfig.I2C_AckCntrl;
	pI2C_Handle->pI2Cx->CR1 |= ( temp << I2C_CR1_ACK);
	temp = 0;

	//7) Configure the Trise
	if(pI2C_Handle->i2cConfig.I2C_Mode == I2C_SM_MODE)
	{
		t_rise = I2C_TRISE_SM;
	}
	else if(pI2C_Handle->i2cConfig.I2C_Mode == I2C_FM_MODE)
	{
		t_rise = I2C_TRISE_FM;
	}

	temp = (pClk * t_rise) + 1;
	pI2C_Handle->pI2Cx->TRISE |= (temp & 0x3F);

	//8) Enable the clock
	I2C_Clock_Peripheral(pI2C_Handle->pI2Cx, ENABLE);
}


void I2C_MasterTx(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t data_len, uint8_t slave_addr)
{
	uint16_t dummy;

	//1) Generate the start bit
	pI2C_Handle->pI2Cx->CR1 |= ( 1 << I2C_CR1_START);

	//2) Confirm that start generation is completed by checking SB bit
	//Cleared by reading the SR1 that we are doing right now
	while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//3) Writing to the DR the address bus with write command. This will clear the SB bit
	I2C_WriteAddrAndReadNWrite(pI2C_Handle, I2C_CMD_WRITE, slave_addr);

	//4) Confirm by checking the ADDR flag bit
	while(! (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));

	//5) Clear the ADDR by reading SR1 and SR2
	dummy = pI2C_Handle->pI2Cx->SR1;
	dummy = pI2C_Handle->pI2Cx->SR2;

	//6) Send the data until the length becomes 0
	while(data_len > 0)
	{
		//6.1) First check if the data register is empty or not
		while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));

		//6.2) Write the data to the Data register
		pI2C_Handle->pI2Cx->DR = *pTxBuffer;

		//6.3) Increment the Tx Buffer and decrement the length
		pTxBuffer++;
		data_len--;
	}

	//7) Check for the TXE = 1
	while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));

	//8) Check for the BTF = 1
	while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_BTF)));

	//9) Generate the stop bit. This will clear the TXE and BTF by hardware
	pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}

void I2C_MasterRx(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t data_len, uint8_t slave_addr)
{
	uint16_t dummy;

	//Start the communication
	pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	//Check the SB Bit and clear it by reading SR1 and writing to DR
	while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//Write the slave address on SDA
	I2C_WriteAddrAndReadNWrite(pI2C_Handle, I2C_CMD_READ, slave_addr);

	//Check if the ADDR Flag is set
	while(!(pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR)));

	if(data_len == 1)
	{
		//ACK = 0
		pI2C_Handle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		//STOP = 1
		pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//Clear the ADDR
		dummy = pI2C_Handle->pI2Cx->SR1;
		dummy = pI2C_Handle->pI2Cx->SR2;

		//Wait till RXNE goes high and if yes then Read the data
		while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));

		*pRxBuffer = pI2C_Handle->pI2Cx->DR;	//RXNE is cleared by reading DR register
	}
	else if(data_len > 1)
	{
		//Clear the ADDR flag by reading SR1 followed by reading SR2
		dummy = pI2C_Handle->pI2Cx->SR1;
		dummy = pI2C_Handle->pI2Cx->SR2;

		//Read the data until length becomes 0
		while(data_len != 0)
		{
			//Check RXNE flag and if its high then read the data
			while(!(pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));

			if(data_len == 2)	//Reading the last two bytes
			{
				//Clear the ACK
				pI2C_Handle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				//Generate stop
				pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			}

			//Send acknowledge
//			pI2C_Handle->pI2Cx->CCR |= (1 << I2C_CR1_ACK);

			// RXNE is cleared by reading DR register
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;

			//Increment buffer and decrement the length
			data_len--;
			pRxBuffer++;
		}
	}

	//Re Enable the acking if it is enabled by the user
	if(pI2C_Handle->i2cConfig.I2C_AckCntrl == I2C_ACK_EN)
		pI2C_Handle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);



}
