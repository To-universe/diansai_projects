#ifndef __APP_ADC_H__
#define __APP_ADC_H__

#include "app_main.h"
#include "arm_math_types.h"
#include <stdint.h>

#define FFT_SIZE 8192
#define ADC_BUFFER_SIZE FFT_SIZE*2
#define sample_rate 1000000

extern volatile uint8_t adc_wait_flag;
extern volatile uint32_t stable_tick;
extern volatile uint32_t now_tick;
extern volatile uint8_t adc_ready;
extern volatile uint8_t adc_sampling;
extern const q15_t hanning_window[FFT_SIZE];
extern arm_rfft_instance_q15 fft_instance;
extern uint32_t amp_response[];

void ADC_wait_stable(void);
void ADC_sample_start(void);
void fft_calc(void);
void adc_to_q15(void);
void response_calc(uint16_t index, uint32_t freq);

#endif
