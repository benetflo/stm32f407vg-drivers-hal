#include "adc.h"
#include <stdio.h>

void init_scan(uint32_t address){

    volatile uint32_t *pAdcCr1Reg = (uint32_t*) address;
    *pAdcCr1Reg |= (1 << 8);
}


void init_adc(uint8_t adc_num){
    volatile uint32_t *pRccApb2Enr = (uint32_t*) RCC_APB2_ENR_ADDR;

    if(adc_num == 1){
		*pRccApb2Enr |= (1 << 8); // activate CLK for ADC1
		init_scan(ADC1_CR1_REG_ADDR);
	}else if(adc_num == 2){
		*pRccApb2Enr |= (1 << 9); // activate CLK for ADC2
		init_scan(ADC2_CR1_REG_ADDR);
	}else if(adc_num == 3){
		*pRccApb2Enr |= (1 << 10); // activate CLK for ADC3
		init_scan(ADC3_CR1_REG_ADDR);
	}else{
		printf("Not a valid ADC number as parameter (1-3)\n");
	}
}
