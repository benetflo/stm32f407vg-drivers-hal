#include "stm32f407g-disc1_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void spi_peri_clk_control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

void spi_init(SPI_Handle_t *pSPIHandle){

	uint32_t tempreg = 0;

	// peripheral clock enable
	spi_peri_clk_control(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be set
		tempreg |= ~(1 << SPI_CR1_BIDI_MODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		// RXONLY bit must be set
		tempreg |= ~(1 << SPI_CR1_RX_ONLY);
	}

	//3. configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void spi_deinit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void spi_send_data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length){
	//BLOCKING API, WILL WAIT UNTIL ALL THE BYTES ARE TRANSMITTED
	while(length > 0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 BIT DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 BIT DFF
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
}

void spi_receieve_data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length){
	//BLOCKING API, WILL WAIT UNTIL ALL THE BYTES ARE TRANSMITTED
	while(length > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 BIT DFF
			//1. load the data from DR to RXbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 BIT DFF
			*(pRxBuffer) = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
	}
}

void spi_irq_config(uint8_t irq_num, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(irq_num <= 31){
			// ISER0 register
			*NVIC_ISER0 |= (1 << irq_num);
		}else if(irq_num > 31 && irq_num < 64){
			// ISER1 register
			*NVIC_ISER1 |= (1 << irq_num % 32);
		}else if(irq_num >= 64 && irq_num < 96){
			// ISER2 register
			*NVIC_ISER3 |= (1 << irq_num % 64);
		}
	}else{
		if(irq_num <= 31){
			// ICER0 register
			*NVIC_ICER0 |= (1 << irq_num);
		}else if(irq_num > 31 && irq_num < 64){
			// ICER1 register
			*NVIC_ICER1 |= (1 << irq_num % 32);
		}else if(irq_num >= 64 && irq_num < 96){
			// ICER2 register
			*NVIC_ICER3 |= (1 << irq_num % 64);
		}
	}
}
void spi_irq_prio_config(uint8_t irq_num, uint32_t irq_prio){

	uint8_t iprx = irq_num / 4;
	uint8_t iprx_section = irq_num % 4;

	// FIRST 4 BITS ARE NOT IMPLEMENTED SO NEED TO SHIFT
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (irq_prio << shift_amount);
}





void spi_irq_handling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;

	// check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE); // if TXE flag is really set: temp1 == 1 else 0
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	// if both temp1 and temp2 == 1, then the interrupt is becauase of setting of TXE flag
	if(temp1 && temp2){
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
			// handle RXNE
			spi_rxne_interrupt_handle(pHandle);
	}

	// check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
			// handle RXNE
			spi_ovr_err_interrupt_handle(pHandle);
	}

	// can check for MODF etc if needed.....
}

// NOTE: This function needs to be called after init functions since you can't change SPI configuration aften SPI peripheral is enabled.
void spi_peripheral_control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


uint8_t spi_send_data_it(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t length){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_RX){
		//1. Save the TX buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = length;
		//2. Mark the SPI state as busy in transmission so that no other code can taker over same
		// SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		//4. Data transmission will be handled by the ISR code (will implement later)
	}
	return state;
}


uint8_t spi_receive_data_it(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t length){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Save the TX buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = length;
		//2. Mark the SPI state as busy in transmission so that no other code can taker over same
		// SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		//4. Data transmission will be handled by the ISR code (will implement later)
	}
	return state;
}


// Helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//2. check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
		//16 BIT DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		//8 BIT DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen){
		// TxLen is zero, so close the spi transmission and inform the application that TX is over.

		//This prevents interruopts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//2. check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
		//16 BIT DFF
		//1. load the data in to the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}else{
		//8 BIT
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if(! pSPIHandle->RxLen){
		// TxLen is zero, so close the spi transmission and inform the application that TX is over.

		//This prevents interruopts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//1. clear the ovr flag
	// if spi peripheral is busy in transmission and the overrun happens code in if will not run
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	// This is a weak implementation.  The application may override this function
}


