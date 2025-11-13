#include <string.h>
#include "stm32f407g-disc1.h"
#include "hw_spi.h"
#include "hw_gpio.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
{
	char user_data[] = "Hello World!\n";

	GPIO_Init(PB, 13, GPIO_AF, GPIO_SPEED_FAST, GPIO_PUPD_DISABLED, GPIO_PUSH_PULL, GPIO_AF5);
	GPIO_Init(PB, 15, GPIO_AF, GPIO_SPEED_FAST, GPIO_PUPD_DISABLED, GPIO_PUSH_PULL, GPIO_AF5);
	GPIO_Init(PB, 12, GPIO_AF, GPIO_SPEED_FAST, GPIO_PUPD_DISABLED, GPIO_PUSH_PULL, GPIO_AF5);

	// button

	SPI_Init(SPI2, FULL_DUPLEX, MASTER, CLK_DIV2, DFF_8BIT, CPOL_LOW, CPHA_LOW, SSM_DI);

	GPIO_Init(PA, 0, GPIO_IN, GPIO_SPEED_FAST, GPIO_PUPD_DISABLED, GPIO_OP_TYPE_DISABLED, GPIO_AF_NONE);

	while(1)
	{
		while( ! GPIO_ReadPin(PA,0) );

		delay();

		SPI_Send(SPI2, user_data);
	}

	return 0;
}
