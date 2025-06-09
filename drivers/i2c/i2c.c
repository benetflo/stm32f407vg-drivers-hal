#include "i2c.h"
#include <stdio.h>


void init_i2c1(void){
    volatile uint32_t *pI2C1_Cr1_Reg = (uint32_t*) I2C1_CR1_REG_ADDR;
    *pI2C1_Cr1_Reg |= (1 << 0); // Peripheral enable I2C1
}

void init_i2c2(void){
    volatile uint32_t *pI2C2_Cr1_Reg = (uint32_t*) I2C2_CR1_REG_ADDR;
    *pI2C2_Cr1_Reg |= (1 << 0); // Peripheral enable I2C2
}

void init_i2c3(void){
    volatile uint32_t *pI2C3_Cr1_Reg = (uint32_t*) I2C3_CR1_REG_ADDR;
    *pI2C3_Cr1_Reg |= (1 << 0); // Peripheral enable I2C3
}

void init_i2c(uint8_t i2c_num){

	volatile uint32_t *pRccApb1Enr = (uint32_t*) RCC_APB1_ENR_ADDR;

    if(i2c_num == 1){
		*pRccApb1Enr |= (1 << 21); // I2C1
		init_i2c1();
	}else if(i2c_num == 2){
		*pRccApb1Enr |= (1 << 22); // I2C2
		init_i2c2();
	}else if(i2c_num == 3){
		*pRccApb1Enr |= (1 << 23); // I2C3
		init_i2c3();
	}else{
		printf("Not a valid I2C number as parameter\n");
	}
}




