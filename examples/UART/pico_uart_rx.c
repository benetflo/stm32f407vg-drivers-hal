#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

#define UART_BAUD_RATE 115200
#define UART_RX_BUFFER_SIZE 256
#define DEBOUNCE_MS 50
#define UART_RX_TIMEOUT_MS 10

char g_rx_buffer[UART_RX_BUFFER_SIZE];


static void uart1_init_config(void)
{
    gpio_set_function(UART1_TX_PIN, UART_FUNCSEL_NUM(uart1, UART1_RX_PIN));
    gpio_set_function(UART1_RX_PIN, UART_FUNCSEL_NUM(uart1, UART1_RX_PIN));

    uart_init(uart1, UART_BAUD_RATE);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);

    // Enable FIFO for better buffering
    uart_set_fifo_enabled(uart1, true);
}

static size_t uart1_receive_message(char* buffer, size_t buffer_size)
{
    if (!uart_is_readable(uart1))
    {
        return 0;
    }

    size_t bytes_read = 0;
    uint64_t start_time = to_us_since_boot(get_absolute_time()) / 1000;

    // Read bytes with timeout
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
                break; // Timeout - no more data coming
            }
            sleep_us(100); // Small delay to avoid busy-waiting
        }
    }

    buffer[bytes_read] = '\0'; // Null-terminate
    return bytes_read;
}

int main()
{
        stdio_init_all();

        uart1_init_config();

        while(1)
        {
                size_t bytes_received = uart1_receive_message(g_rx_buffer, UART_RX_BUFFER_SIZE);

                if (bytes_received > 0)
                {
                        printf("[RX] UART1: %s", g_rx_buffer);

                        // Add newline if message doesn't end with one
                        if (g_rx_buffer[bytes_received - 1] != '\n')
                        {
                                printf("\n");
                        }
                        sleep_ms(1);
                }
        }

        return 0;
}
