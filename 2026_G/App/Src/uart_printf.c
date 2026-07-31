#include "uart_printf.h"

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t cr = '\r';
        HAL_UART_Transmit(&huart5, &cr, 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int __io_getchar(void)
{
    uint8_t c = 0;
    HAL_UART_Receive(&huart5, &c, 1, HAL_MAX_DELAY);
    return (int)c;
}
