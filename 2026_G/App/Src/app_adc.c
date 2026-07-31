#include "app_adc.h"

uint16_t adc_buffer[ADC_SIZE];

static volatile uint8_t frame_ready = 0U;

void adc_signal_frame_ready(void)
{
    frame_ready = 1U;
}

uint8_t adc_is_frame_ready(void)
{
    return frame_ready;
}

void adc_clear_frame_ready(void)
{
    frame_ready = 0U;
}

const uint16_t* adc_get_buffer(void)
{
    return adc_buffer;
}
