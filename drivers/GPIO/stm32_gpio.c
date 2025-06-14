#include <stdio.h>
#include "stm32_gpio.h"

void init_gpio_port_clk(uint8_t port_num){

	if(port_num > 10){
		printf("Not a valid GPIO port number, consult the data sheet.\n");
		return;
	}
	volatile uint32_t *pRccAhbEnr = (uint32_t *)RCC_AHB1ENR;
	*pRccAhbEnr |= (1 << port_num);
}


