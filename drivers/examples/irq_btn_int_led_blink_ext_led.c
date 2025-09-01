

#include <stdint.h>
#include <string.h>
#include "stm32f407g-disc1.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay(){
	// approx 200ms when SYSCLK is 16MHz
	for(int i = 0; i < 500000/2; i++){
		;
	}
}

/*
 * NOTE: The red LED that is used to warn for overcurrent on VBUS
 * will be lit since it is connected to the same pin as the button (PD5)
 */

int main(void)
{
    GPIO_Handle_t gpio_led, gpio_btn, gpio_led2;


    // Initialize GPIO structures to 0 to avoid garbage values (set memory)
    memset(&gpio_led, 0, sizeof(gpio_led));
    memset(&gpio_btn, 0, sizeof(gpio_btn));
    memset(&gpio_led2, 0, sizeof(gpio_led2));

    gpio_led.pGPIOx = GPIOD;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    gpio_led2.pGPIOx = GPIOA;
	gpio_led2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	gpio_led2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	gpio_led2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    gpio_btn.pGPIOx = GPIOD;
    gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    //gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;   // only output
    gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    gpio_peri_clk_control(GPIOD, ENABLE);
    gpio_init(&gpio_led);
    gpio_init(&gpio_btn);
    gpio_peri_clk_control(GPIOA, ENABLE);
    gpio_init(&gpio_led2);


    gpio_irq_prio_config(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
    gpio_irq_config(IRQ_NO_EXTI9_5 ,ENABLE);

    while(1){
    	gpio_write_pin(GPIOA, GPIO_PIN_NO_1, 1);
    	delay();
    	gpio_write_pin(GPIOA, GPIO_PIN_NO_1, 0);
    	delay();

    }

	return 0;
}

// can be found in startup_stm32f407vgtx.s
void EXTI9_5_IRQHandler(void){
	delay();
	gpio_irq_handling(GPIO_PIN_NO_5); // clear pending event from EXTI line
	gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
}
