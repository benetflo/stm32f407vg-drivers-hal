#ifndef INC_STM32F407G_DISC1_SPI_DRIVER_H_
#define INC_STM32F407G_DISC1_SPI_DRIVER_H_

#include "stm32f407g-disc1.h"


typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct {
	SPI_RegDef_t	*pSPIx; // holds the base address of SPIx(0, 1, 2) peripheral
	SPI_Config_t    SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;


/*
 * SPI RELATED MACROS
 */

// SPI application states
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

// pOSSIPLE spi APPLICATION EVENTS
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

// SPI_SelectMode
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0	//default

// SPI_BusConfig
#define SPI_BUS_CONFIG_FD	1 // full duplex
#define SPI_BUS_CONFIG_HD	2 // half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3 // simplex rx only

// SPI_SclkSpeed (consult reference manual CR1 register->BR)
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

// SPI_DFF
#define SPI_DFF_8BITS 	0 //default
#define SPI_DFF_16BITS	1

// SPI_CPOL
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

// SPI_CPHA
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

// SPI_SSM
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0

// SPI RELATED STATUS FLAGS (masking details)
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)		// Transmit buffer not empty
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)		// Receive buffer not empty
#define SPI_BUSY_FLAG	(1 << SPI_SR_BUSY)		// Checks if peripheral is busy

void spi_peri_clk_control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void spi_init(SPI_Handle_t *pSPIHandle);
void spi_deinit(SPI_RegDef_t *pSPIx);

void spi_send_data(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t length);
void spi_receive_data(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t length);

uint8_t spi_send_data_it(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t length);
uint8_t spi_receive_data_it(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t length);

void spi_irq_config(uint8_t irq_num, uint8_t EnOrDi);
void spi_irq_prio_config(uint8_t irq_num, uint32_t irq_prio);
void spi_irq_handling(SPI_Handle_t *pHandle);


void spi_peripheral_control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


// Application callback (application needs to impliment this)
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407G_DISC1_SPI_DRIVER_H_ */

