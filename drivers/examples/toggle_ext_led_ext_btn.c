#include <stdint.h>
#include "stm32f407g-disc1.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay(){
	for(int i = 0; i < 500000/2; i++){
		;
	}
}

int main(void)
{
    GPIO_Handle_t gpio_led;
    gpio_led.pGPIOx = GPIOA;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    gpio_peri_clk_control(GPIOA, ENABLE);
    gpio_init(&gpio_led);

    GPIO_Handle_t gpio_btn;
    gpio_btn.pGPIOx = GPIOB;
    gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    //gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;   // only output
    gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    gpio_peri_clk_control(GPIOB, ENABLE);
    gpio_init(&gpio_btn);



	/* Loop forever */
	while(1){
		if(gpio_read_pin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED){
			delay(); // debounce
			gpio_toggle_pin(GPIOA, GPIO_PIN_NO_8);
		}
	}

}
