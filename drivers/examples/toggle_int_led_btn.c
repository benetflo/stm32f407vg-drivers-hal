#include <stdint.h>
#include "stm32f407g-disc1.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(){
	for(int i = 0; i < 500000/2; i++){
		;
	}
}

int main(void)
{
    GPIO_Handle_t gpio_led;
    gpio_led.pGPIOx = GPIOD;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Handle_t gpio_btn;
    gpio_btn.pGPIOx = GPIOA;
    gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    //gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;   // only output
    gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    gpio_peri_clk_control(GPIOA, ENABLE);
    gpio_peri_clk_control(GPIOD, ENABLE);
    gpio_init(&gpio_led);
    gpio_init(&gpio_btn);



	/* Loop forever */
	while(1){
		if(gpio_read_pin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED){
			delay(); // debounce
			gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
		}
	}

}
