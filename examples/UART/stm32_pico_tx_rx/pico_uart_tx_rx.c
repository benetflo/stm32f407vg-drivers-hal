#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>

#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

#define UART_BAUD_RATE 115200
#define UART_RX_BUFFER_SIZE 256
#define DEBOUNCE_MS 50
#define UART_RX_TIMEOUT_MS 10

char g_rx_buffer[UART_RX_BUFFER_SIZE];

// init UART1
static void uart1_init_config(void)
{
    // Sätt TX och RX pins
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);

    uart_init(uart1, UART_BAUD_RATE);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    // Enable FIFO for better buffering
    uart_set_fifo_enabled(uart1, true);
}

// receive msg with timeout
static size_t uart1_receive_message(char* buffer, size_t buffer_size)
{
    if (!uart_is_readable(uart1))
    {
        return 0;
    }

    size_t bytes_read = 0;
    uint64_t start_time = to_us_since_boot(get_absolute_time()) / 1000;

    // read bytes with timeout
    while (bytes_read < buffer_size - 1)
    {
        if (uart_is_readable(uart1))
        {
            buffer[bytes_read++] = uart_getc(uart1);
            start_time = to_us_since_boot(get_absolute_time()) / 1000; // Reset timeout
        }
        else
        {
            uint64_t now = to_us_since_boot(get_absolute_time()) / 1000;
            if (now - start_time > UART_RX_TIMEOUT_MS)
            {
                break; // Timeout - no more bytes
            }
            sleep_us(100); 
        }
    }

    buffer[bytes_read] = '\0'; // Null-terminate
    return bytes_read;
}

// send msg via UART1
static void uart1_send_message(const char * message)
{
    if (uart_is_writable(uart1))
    {
        uart_puts(uart1, message);
    }
}

int main()
{
    stdio_init_all();

    uart1_init_config();

    const char * msg = "ACK\n";

    while (1)
    {
        // Read msg from STM32
        size_t bytes_received = uart1_receive_message(g_rx_buffer, UART_RX_BUFFER_SIZE);

        if (bytes_received > 0)
        {
            printf("[RX] UART1: %s", g_rx_buffer);

            // add newline if msg does not have it
            if (g_rx_buffer[bytes_received - 1] != '\n')
            {
                printf("\n");
            }

            // send ACK back to STM32
            uart1_send_message(msg);

            sleep_ms(1);
        }
    }

    return 0;
}
