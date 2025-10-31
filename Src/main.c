// BLINK EXAMPLE WITH THE ABSTRACTION LAYER

#include "hw_gpio.h"
#include "stdio.h"
#include <string.h>

void delay(uint32_t count)
{
    for(volatile uint32_t i = 0; i < count; i++);
}

void blink(void)
{
	GPIO_TogglePin(PA, 0);
	delay(50000);
}

int main()
{

    GPIO_Init(PA, 0, GPIO_OUT, GPIO_SPEED_FAST, GPIO_PUPD_DISABLED, GPIO_PUSH_PULL, GPIO_AF_NONE);
    GPIO_IRQInit(PA, 1, IRQ_MODE_FALLING, GPIO_SPEED_FAST, GPIO_PULL_UP, 1, blink);

    while(1)
    {
    	;
    }

    return 0;
}

