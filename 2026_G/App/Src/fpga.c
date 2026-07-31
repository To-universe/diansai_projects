#include "fpga.h"
#include "adc.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"

static int16_t buffer[FPGA_SAMPLE_COUNT];
static volatile uint8_t data_ready = 0U;

void fpga_reset(void)
{
    HAL_GPIO_WritePin(FPGA_RSTN_GPIO_Port, FPGA_RSTN_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(FPGA_RSTN_GPIO_Port, FPGA_RSTN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void fpga_start_capture(void)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = 0x07;
    tx_buffer[1] = 12;

    if (HAL_SPI_Transmit(&hspi2, tx_buffer, 2, 1000) != HAL_OK) {
        Error_Handler();
    }
}

void fpga_signal_data_ready(void)
{
    data_ready = 1U;
}

uint8_t fpga_is_data_ready(void)
{
    return data_ready;
}

void fpga_clear_data_ready(void)
{
    data_ready = 0U;
}

uint8_t fpga_receive(void)
{
    uint8_t rx_buffer[8193];
    const uint32_t rx_size = (uint32_t)FPGA_SAMPLE_COUNT * 2U + 1U;

    data_ready = 0U;

    if (HAL_SPI_Receive(&hspi2, rx_buffer, rx_size, 5000) != HAL_OK) {
        Error_Handler();
        return 0U;
    }

    if (rx_buffer[rx_size - 1U] != 0x21) {
        Error_Handler();
        return 0U;
    }

    for (uint32_t i = 0U; i < FPGA_SAMPLE_COUNT; i++) {
        uint8_t hdata = rx_buffer[2U * i];
        uint8_t ldata = rx_buffer[2U * i + 1U];
        uint16_t data = (((uint16_t)hdata) << 8) | ldata;
        buffer[i] = *(int16_t *)&data;
    }

    return 1U;
}

const int16_t *fpga_get_buffer(void)
{
    return buffer;
}
