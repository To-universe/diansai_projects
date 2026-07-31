#ifndef __APP_ADC_H__
#define __APP_ADC_H__

#include <stdint.h>

#define ADC_SIZE        4096U
#define FS_MCU              2000000.0f

extern uint16_t adc_buffer[ADC_SIZE];

void adc_signal_frame_ready(void);
uint8_t adc_is_frame_ready(void);
void adc_clear_frame_ready(void);
const uint16_t* adc_get_buffer(void);

#endif
