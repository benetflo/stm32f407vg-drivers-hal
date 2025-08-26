#include <stdint.h>
#include "stm32f407g-disc1.h"

void delay(){
	for(int i = 0; i < 500000; i++){
		;
	}
}

int main(void)
{
    GPIO_Handle_t gpio_led;
    gpio_led.pGPIOx = GPIOA;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    gpio_peri_clk_control(GPIOA, ENABLE);
    gpio_init(&gpio_led);


	/* Loop forever */
	while(1){
		gpio_write_pin(GPIOA , GPIO_PIN_NO_2, 1);
		delay();
		gpio_write_pin(GPIOA , GPIO_PIN_NO_2, 0);
		delay();
	}

}
