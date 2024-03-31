/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Mar 17, 2024
 *      Author: Rakesh Ahuja
 */

#ifndef SRC_STM32F407XX_I2C_DRIVER_H_
#define SRC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct{
	uint8_t I2C_Mode;
	uint8_t I2C_SclSpeed;
	uint8_t I2C_AckCntrl;
	uint8_t I2C_FMDutyCycle;
	uint8_t I2C_AddressMode;
	uint16_t I2C_DeviceAddress;
}I2C_Config_t;

typedef struct{
	I2C_Config_t i2cConfig;
	I2C_RegDef_t *pI2Cx;
}I2C_Handle_t;

/*
 * TODO 2(I2C.h) - Macros for enable/disable
 */
#define I2C_DISABLE					0
#define I2C_ENABLE					1

/*
 * TODO 3(I2C.h) - Ack Control
 */
#define I2C_ACK_DI					0
#define I2C_ACK_EN					1

/*
 * TODO 4(I2C.h) - Duty cycle of fast mode
 */
#define I2C_FM_DUTY_CYCLE_2			0
#define I2C_FM_DUTY_CYCLE_16_9		1

/*
 * TODO 6(I2C.h) - Address mode
 */
#define I2C_7_BIT_ADD				0
#define I2C_10_BIT_ADD				1

/*
 * TODO 6(I2C.h) - Clock @Scl_speed
 */
#define I2C_SPEED_SM				100000
#define I2C_SPEED_FM2K				200000
#define I2C_SPEED_FM4K				400000

/*
 * TODO 7(I2C.h) - Clock speed
 */
#define I2C_SM_MODE					0
#define I2C_FM_MODE					1

//TODO 7(I2C.h) - Macros for reading and writing commands
#define I2C_CMD_READ				1
#define I2C_CMD_WRITE				0

//TODO 8(I2C.h) - Header for 10 bit data
#define I2C_HEADER_10_ADD			0x11110000

//TODO 9: Value of Trise
#define I2C_TRISE_SM				(1/1000000U)
#define I2C_TRISE_FM				(3/10000000U)
/*
 * Functions Declarations
 */

void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterTx(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t data_len, uint8_t slave_addr);
void I2C_MasterRx(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t data_len, uint8_t slave_addr);;

void I2C_SlaveTx(I2C_Handle_t *pI2C_Handle);
void I2C_SlaveRx(I2C_Handle_t *pI2C_Handle);


#endif /* SRC_STM32F407XX_I2C_DRIVER_H_ */
