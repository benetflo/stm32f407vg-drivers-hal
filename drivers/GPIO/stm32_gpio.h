#ifndef STM32_GPIO_H
#define STM32_GPIO_H

#include <stdint.h>

#define RCC_BASE_ADDR              0x40023800UL
#define RCC_AHB1_OFFSET            0x30UL
#define RCC_AHB1ENR                (RCC_BASE_ADDR + RCC_AHB1_OFFSET)

void init_gpio_port_clk(uint8_t port_num);



#endif
