#ifndef __CALC_H__
#define __CALC_H__

#include "arm_math_types.h"
#include "voltage.h"
#include <stdint.h>

#define CALC_FRAME_COUNT    5U

typedef struct {
    float32_t vrms[CALC_FRAME_COUNT];
    float32_t vpp[CALC_FRAME_COUNT];
    float32_t harmonic_amps[CALC_FRAME_COUNT][PEAK_MAX_COUNT];
    uint8_t   harmonic_orders[CALC_FRAME_COUNT][PEAK_MAX_COUNT];
    uint8_t   harmonic_order_count[CALC_FRAME_COUNT];
    float32_t f0;
    float32_t f0_used[CALC_FRAME_COUNT];
    float32_t waveform[CALC_FRAME_COUNT][WAVEFORM_SIZE];
    uint8_t   count;
} calc_accumulator_t;

void calc_init(calc_accumulator_t *acc);
uint8_t calc_accumulate(calc_accumulator_t *acc, const vol_result_t *vol);
void calc_finalize(const calc_accumulator_t *acc,
                   float32_t *vrms_mean,
                   float32_t *vpp_mean,
                   float32_t *f0_mean,
                   float32_t *harmonic_means,
                   uint8_t  *valid_counts,
                   uint8_t  *harmonic_orders,
                   uint8_t  *harmonic_order_count,
                   float32_t *waveform_mean);

#endif
