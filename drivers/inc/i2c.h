#ifndef INC_STM32F407G_DISC1_I2C_DRIVER_H_
#define INC_STM32F407G_DISC1_I2C_DRIVER_H_

#include "stm32f407g-disc1.h"

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;	//user decides this
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000	//100 kHz
#define I2C_SCL_SPEED_FM4K	400000	//400 kHz
#define I2C_SCL_SPEED_FM2K	200000	//200 kHz

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


// I2C RELATED STATUS FLAGS (masking details)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)		// Transmit buffer not empty
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)		// Receive buffer not empty
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

void i2c_peri_clk_control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void i2c_init(I2C_Handle_t *pI2CHandle);
void i2c_deinit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Data Send and Receive
 */
void i2c_master_send_data(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
void i2c_master_recieve_data(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr);

void i2c_irq_config(uint8_t irq_num, uint8_t EnOrDi);
void i2c_irq_prio_config(uint8_t irq_num, uint32_t irq_prio);


void i2c_peripheral_control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

// Application callback (application needs to impliment this)
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407G_DISC1_I2C_DRIVER_H_ */
