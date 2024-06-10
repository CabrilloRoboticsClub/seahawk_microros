#include <stdio.h>
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <hardware/uart.h>

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    stdio_init_all();
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
{
    for (size_t i = 0; i < len; i++)
    {
        if (buf[i] != putchar(buf[i]))
        {
            *errcode = 1;
            return i;
        }
    }
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++)
    {
        int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0)
        {
            *errcode = 1;
            return i;
        }

        int character = getchar_timeout_us(elapsed_time_us);
        if (character == PICO_ERROR_TIMEOUT)
        {
            *errcode = 1;
            return i;
        }
        buf[i] = character;
    }
    return len;
}