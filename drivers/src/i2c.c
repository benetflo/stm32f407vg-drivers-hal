#include "stm32f407g-disc1_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAdressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAdressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1; // First shift the slaveaddr by 1 bit, making space for the read/write bit.
	SlaveAddr &= ~(1);			// SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;	// type cast to void otherwise compiler will issue "unused variable" error
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

void i2c_peri_clk_control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(){
	return;
}


uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp-8];
	}

	// for APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1p = 1;
	}else{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;


}

void i2c_init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0;

	// ACK control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); // mask all the bits except first five bits

	// store the slave address in own address register OAR
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14); // bit 14 needs to be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode. Configure mode in CCR reg 15 bit. Is SM as default
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);		// only 12 bits are valid, mask out other bits except first 12 bits
	}else{
		// mode is fast mode
		tempreg |= (1 << 15); // fast mode is configured
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);		// only 12 bits are valid, mask out other bits except first 12 bits
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	//TRISE configuration

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode.
		tempreg = RCC_GetPCLK1Value() / 1000000U + 1;

	}else{
		// mode is fast mode.
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

	//1. configure the CR1 register and the ACK control bit -> Enables or disables the acking
	//2. configure the FREQ field of CR2, because I2C uses the FREQ field to decide what is the frequency of APB1
	//3. Calculate the APB1 clock and store that value in the CR2 register FREQ field
	//4. Program the device own address into the OAR1 register
	//5. Calculate the CCR value and store it in the CCR field of the CCR register (important to produce different serial clock speeds)
	//6. TRISE calculation
}

void i2c_deinit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void i2c_master_send_data(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr){

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// NOTE: Until SB is cleared SCL will be stretched (pulled to low)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAdressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its sofware sequence
	// NOTE: until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send the data until Len becomes 0
	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait till TXE is set == buffer is empty
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	//7. When Len becomes 0, wait for TXE=1 and BTF=1 before generating the STOP condition
	// NOTE: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// NOTE: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void i2c_master_recieve_data(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr){
	// 1. Generate the START condition

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// NOTE: Until SB is cleared SCL will be stretched (pulled to LOW)

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1

	// start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//procedure to read only 1 byte from slave
	if(Len == 1){
		// Disable Acking

		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Wait until RXNE becomes 1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		// Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// Read data in to buffer

		return;
	}

	//procedure to read data from slave when Len > 1
	if(Len > 1){
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Read the data until Len becomes 0
		for(uint32_t i = Len; i > 0; i--){
			// wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(i == 2){			// if last 2 bytes are remaining
				//clear the ACK bit

				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}
			// read the data from data register in to buffer

			//increment the buffer address


		}
	}
	// re-enable ACKing
}
