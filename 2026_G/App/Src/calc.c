#include "calc.h"
#include <string.h>

void calc_init(calc_accumulator_t *acc)
{
    memset(acc, 0, sizeof(calc_accumulator_t));
}

uint8_t calc_accumulate(calc_accumulator_t *acc, const vol_result_t *vol)
{
    if (acc->count >= CALC_FRAME_COUNT) {
        return 1U;
    }

    acc->vrms[acc->count] = vol->vrms;
    acc->vpp[acc->count]  = vol->vpp;
    if(acc->count == 0){
        acc->f0 = vol->f0;
    }
    acc->f0_used[acc->count]   = vol->f0_used;

    uint8_t component_count = vol->harmonic_order_count;
    if (component_count > PEAK_MAX_COUNT) {
        component_count = PEAK_MAX_COUNT;
    }
    acc->harmonic_order_count[acc->count] = component_count;

    for (uint8_t i = 0U; i < PEAK_MAX_COUNT; i++) {
        acc->harmonic_amps[acc->count][i] = vol->harmonic_amps[i];
        acc->harmonic_orders[acc->count][i] = vol->harmonic_orders[i];
    }

    for (uint16_t k = 0U; k < WAVEFORM_SIZE; k++) {
        acc->waveform[acc->count][k] = vol->waveform[k];
    }

    acc->count++;

    return (acc->count >= CALC_FRAME_COUNT) ? 1U : 0U;
}

void calc_finalize(const calc_accumulator_t *acc,
                   float32_t *vrms_mean,
                   float32_t *vpp_mean,
                   float32_t *f0_mean,
                   float32_t *harmonic_means,
                   uint8_t  *valid_counts,
                   uint8_t  *harmonic_orders,
                   uint8_t  *harmonic_order_count,
                   float32_t *waveform_mean)
{
    float32_t sum_vrms = 0.0f;
    float32_t sum_vpp  = 0.0f;
    float32_t sum_f0   = 0.0f;
    uint8_t best_order_frame = 0U;
    uint8_t best_order_count = 0U;

    for (uint8_t i = 0U; i < CALC_FRAME_COUNT; i++) {
        sum_vrms += acc->vrms[i];
        sum_vpp  += acc->vpp[i];
        sum_f0   += acc->f0_used[i];
        if (acc->harmonic_order_count[i] > best_order_count) {
            best_order_count = acc->harmonic_order_count[i];
            best_order_frame = i;
        }
    }

    *vrms_mean = sum_vrms / (float32_t)CALC_FRAME_COUNT;
    *vpp_mean  = sum_vpp  / (float32_t)CALC_FRAME_COUNT;
    *f0_mean   = sum_f0   / (float32_t)CALC_FRAME_COUNT;

    *harmonic_order_count = best_order_count;
    for (uint8_t i = 0U; i < PEAK_MAX_COUNT; i++) {
        harmonic_orders[i] = (i < best_order_count) ? acc->harmonic_orders[best_order_frame][i] : 0U;
    }

    for (uint8_t order = 0U; order < PEAK_MAX_COUNT; order++) {
        float32_t sum_amp = 0.0f;
        uint8_t valid = 0U;
        uint8_t target_order = harmonic_orders[order];

        if (target_order > 0U) {
            for (uint8_t frame = 0U; frame < CALC_FRAME_COUNT; frame++) {
                for (uint8_t component = 0U;
                     component < acc->harmonic_order_count[frame] && component < PEAK_MAX_COUNT;
                     component++) {
                    if (acc->harmonic_orders[frame][component] == target_order &&
                        acc->harmonic_amps[frame][component] > 0.0f) {
                        sum_amp += acc->harmonic_amps[frame][component];
                        valid++;
                        break;
                    }
                }
            }
        }

        valid_counts[order] = valid;
        harmonic_means[order] = (valid > 0U) ? sum_amp / (float32_t)valid : 0.0f;
    }

    for (uint16_t k = 0U; k < WAVEFORM_SIZE; k++) {
        float32_t sum = 0.0f;
        for (uint8_t frame = 0U; frame < CALC_FRAME_COUNT; frame++) {
            sum += acc->waveform[frame][k];
        }
        waveform_mean[k] = sum / (float32_t)CALC_FRAME_COUNT;
    }
}
