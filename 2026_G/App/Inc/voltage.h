#ifndef __VOLTAGE_H__
#define __VOLTAGE_H__

#include "arm_math_types.h"
#include "fft.h"
#include <stdint.h>

#define METRIC_SIZE             4000U
#define ADC_VREF                3.3f
#define ADC_FULL_SCALE          4095.0f
#define INPUT_SCALE             1.0f
#define VOL_HARMONIC_ORDER_MAX  HARMONIC_ORDER_MAX
#define VOL_AMP_BY_ORDER_SIZE   (VOL_HARMONIC_ORDER_MAX + 1U)
#define WAVEFORM_SIZE           960U

typedef struct {
    float32_t vrms;
    float32_t vpp;
    float32_t f0;
    float32_t f0_used;
    float32_t dc_offset;
    float32_t harmonic_amps[PEAK_MAX_COUNT];
    uint8_t harmonic_order_count;
    uint8_t harmonic_orders[PEAK_MAX_COUNT];
    float32_t waveform[WAVEFORM_SIZE];
} vol_result_t;

float32_t voltage_get_sample_to_volt(void);
void voltage_set_sample_to_volt(float32_t k);
void voltage_compute(const int16_t *fpga_data, const fft_result_t *fft, vol_result_t *result);

#endif
