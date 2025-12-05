#include <stdio.h>
#include <string.h>
#include "hw_uart.h"
#include "hw_gpio.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

int main(void)
{
    // Green onboard LED
    GPIO_Init(PD, 12, GPIO_OUT, GPIO_SPEED_FAST, GPIO_PUSH_PULL, GPIO_PUPD_DISABLED, GPIO_AF_NONE);

    // Onboard USER button
    GPIO_Init(PA, 0, GPIO_IN, GPIO_SPEED_FAST, GPIO_OP_TYPE_DISABLED, GPIO_PUPD_DISABLED, GPIO_AF_NONE);

    // USART2 TX 
    GPIO_Init(PA, 2, GPIO_AF, GPIO_SPEED_FAST, GPIO_PUSH_PULL, GPIO_OP_TYPE_DISABLED, GPIO_AF7);

    // USART2 RX
    GPIO_Init(PA, 3, GPIO_AF, GPIO_SPEED_FAST, GPIO_PUSH_PULL, GPIO_OP_TYPE_DISABLED, GPIO_AF7);

    USART_Handle_t uart_t = usart_init(
        USART2,
		115200,
		HW_FLOW_CTRL_NONE,
		TX_AND_RX,
		STOP_BIT_1,
		WORDLEN_8,
		PARITY_DISABLE
    );

    char s_msg[1024] = "Hello Pico\n";
    char r_msg[6];

    while (1)
    {
        // wait for button press
        while (!gpio_read_pin(GPIOA, 0));

        delay();

        // send msg to Pico
        USART_Send(&uart_t, (uint8_t*)s_msg, strlen(s_msg));

        // recieve msg until newline or max 5 bytes
        uint8_t ch;
        int idx = 0;

        do {
            USART_Recieve(&uart_t, &ch, 1);
            r_msg[idx++] = ch;
        } while (ch != '\n' && idx < 5);

        r_msg[idx] = '\0';

        // Light GREEN onboard LED if Pico returns "ACK\n"
        if (strcmp(r_msg, "ACK\n") == 0)
        {
            gpio_write_pin(PD, 12, GPIO_HIGH);
        }
    }

    return 0;
}
