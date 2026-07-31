#include "voltage.h"
#include "app_adc.h"
#include "arm_math_types.h"
#include "calibration_vpp.h"
#include "dsp/fast_math_functions.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#define LS_MAX_DIM      (1U + 2U * PEAK_MAX_COUNT)

static float32_t g_sample_to_volt = 4.383015e-6f;

float32_t voltage_get_sample_to_volt(void)
{
    return g_sample_to_volt;
}

void voltage_set_sample_to_volt(float32_t k)
{
    if (k > 0.0f) {
        g_sample_to_volt = k;
    }
}

static uint8_t solve_linear_system(float32_t a[LS_MAX_DIM][LS_MAX_DIM],
                                    float32_t b[LS_MAX_DIM],
                                    float32_t x[LS_MAX_DIM],
                                    uint8_t dim)
{
    float32_t pivot, factor, tmp;

    for (uint8_t i = 0U; i < dim; i++) {
        uint8_t pivot_row = i;
        float32_t max_abs = fabsf(a[i][i]);
        for (uint8_t r = i + 1U; r < dim; r++) {
            float32_t v = fabsf(a[r][i]);
            if (v > max_abs) {
                max_abs = v;
                pivot_row = r;
            }
        }

        if (max_abs < 1.0e-9f) {
            return 0U;
        }

        if (pivot_row != i) {
            for (uint8_t c = i; c < dim; c++) {
                tmp = a[i][c];
                a[i][c] = a[pivot_row][c];
                a[pivot_row][c] = tmp;
            }
            tmp = b[i];
            b[i] = b[pivot_row];
            b[pivot_row] = tmp;
        }

        pivot = a[i][i];
        for (uint8_t r = i + 1U; r < dim; r++) {
            factor = a[r][i] / pivot;
            a[r][i] = 0.0f;
            for (uint8_t c = i + 1U; c < dim; c++) {
                a[r][c] -= factor * a[i][c];
            }
            b[r] -= factor * b[i];
        }
    }

    for (int32_t i = (int32_t)dim - 1; i >= 0; i--) {
        float32_t sum = b[i];
        for (uint8_t c = (uint8_t)(i + 1); c < dim; c++) {
            sum -= a[i][c] * x[c];
        }
        x[i] = sum / a[i][i];
    }

    return 1U;
}

/*
 * Harmonic decomposition via least-squares fitting.
 * Returns component_count.  Outputs:
 *   I[component_count], Q[component_count], order_list[component_count]
 *   dc_offset  — fitted DC level (volts)
 */
static uint8_t harmonic_decompose(const int16_t *fpga_data,
                                   const fft_result_t *fft,
                                   float32_t sample_to_volt,
                                   float32_t I[PEAK_MAX_COUNT],
                                   float32_t Q[PEAK_MAX_COUNT],
                                   uint8_t  order_list[PEAK_MAX_COUNT],
                                   float32_t *dc_offset)
{
    uint8_t component_count = 0U;

    for (uint8_t i = 0U; i < fft->peak_count; i++) {
        uint8_t order = fft->peak_harmonic_order[i];
        if (order > 0U) {
            order_list[component_count] = order;
            component_count++;
        }
    }

    if (component_count == 0U) {
        return 0U;
    }

    float32_t f0_for_fit = fft->measured_f0;
    if (f0_for_fit <= 0.0f) {
        f0_for_fit = fft->fitted_f0;
    }

    uint8_t dim = (uint8_t)(1U + 2U * component_count);
    float32_t normal[LS_MAX_DIM][LS_MAX_DIM] = {{0.0f}};
    float32_t rhs[LS_MAX_DIM] = {0.0f};
    float32_t coef[LS_MAX_DIM] = {0.0f};
    float32_t row[LS_MAX_DIM] = {0.0f};
    float32_t base_phase_step = 2.0f * PI * f0_for_fit / FS;

    for (uint16_t j = 0U; j < METRIC_SIZE; j++) {
        float32_t v = (float32_t)fpga_data[j] * sample_to_volt;
        float32_t base_phase = base_phase_step * (float32_t)j;

        row[0] = 1.0f;
        for (uint8_t i = 0U; i < component_count; i++) {
            float32_t phase = base_phase * (float32_t)order_list[i];
            row[1U + 2U * i] = arm_cos_f32(phase);
            row[2U + 2U * i] = arm_sin_f32(phase);
        }

        for (uint8_t r = 0U; r < dim; r++) {
            rhs[r] += row[r] * v;
            for (uint8_t c = 0U; c < dim; c++) {
                normal[r][c] += row[r] * row[c];
            }
        }
    }

    if (!solve_linear_system(normal, rhs, coef, dim)) {
        return 0U;
    }

    *dc_offset = 0;

    for (uint8_t i = 0U; i < component_count; i++) {
        I[i] = coef[1U + 2U * i];
        Q[i] = coef[2U + 2U * i];
    }

    return component_count;
}

/*
 * harmonic_decompose_v2  —  spectrum-based (no linear regression).
 *
 * For a periodic signal with discrete spectral lines the FFT directly
 * yields I/Q coefficients.  With arm_rfft_q15 (1/N scaling) and a
 * Hanning window (coherent gain 0.5):
 *
 *   X_scaled[k]  =  ¼·(I − j·Q)         (positive bin k)
 *   ⇒  I = 4·Re{X[k]},   Q = −4·Im{X[k]}
 *
 * Amplitude (in ADC-count units) = 4 · magnitude[peak_bin].
 * Multiply by sample_to_volt to get physical volts.
 */
static uint8_t harmonic_decompose_v2(const int16_t *fpga_data,
                                      const fft_result_t *fft,
                                      float32_t sample_to_volt,
                                      float32_t I[PEAK_MAX_COUNT],
                                      float32_t Q[PEAK_MAX_COUNT],
                                      uint8_t  order_list[PEAK_MAX_COUNT],
                                      float32_t *dc_offset)
{
    float32_t dc_raw = 0.0f;
    for (uint16_t j = 0U; j < METRIC_SIZE; j++) {
        dc_raw += (float32_t)fpga_data[j];
    }
    *dc_offset = dc_raw / (float32_t)METRIC_SIZE * sample_to_volt;

    uint8_t component_count = 0U;
    for (uint8_t i = 0U; i < fft->peak_count; i++) {
        uint8_t order = fft->peak_harmonic_order[i];
        if (order > 0U) {
            order_list[component_count] = order;
            component_count++;
        }
    }

    if (component_count == 0U || fft->magnitude == NULL || fft->fft_output == NULL) {
        return 0U;
    }

    for (uint8_t i = 0U; i < component_count; i++) {
        uint16_t pk = fft->peak_bin[i];

        float32_t re = (float32_t)fft->fft_output[2U * pk];
        float32_t im = (float32_t)fft->fft_output[2U * pk + 1U];

        /* I = 4·re,  Q = −4·im   (in ADC-count units)  →  convert to volts */
        float32_t component_freq = fft->fitted_peak_freq[i];
        if (component_freq <= 0.0f) {
            component_freq = fft->peak_interp_freq[i];
        }
        float32_t freq_corr = cal_corr(component_freq);
        // freq_corr = 1;

        I[i] =  4.0f * re * sample_to_volt * freq_corr;
        Q[i] = -4.0f * im * sample_to_volt * freq_corr;
    }

    return component_count;
}

void voltage_compute(const int16_t *fpga_data, const fft_result_t *fft, vol_result_t *result)
{
    memset(result, 0, sizeof(vol_result_t));

    if (fft->peak_count == 0U || fft->fitted_f0 <= 0.0f) {
        return;
    }

    /* pick the more precise f0 estimate */
    result->f0 = fft->fitted_f0;
    result->f0_used = fft->measured_f0;
    if (result->f0_used <= 0.0f) {
        result->f0_used = fft->fitted_f0;
    }

    float32_t sample_to_volt = g_sample_to_volt;
    //
    float32_t I[PEAK_MAX_COUNT] = {0.0f};
    float32_t Q[PEAK_MAX_COUNT] = {0.0f};
    uint8_t order_list[PEAK_MAX_COUNT] = {0U};
    float32_t dc = 0.0f;

    uint8_t component_count = harmonic_decompose_v2(fpga_data, fft, sample_to_volt,
                                                   I, Q, order_list, &dc);
    if (component_count == 0U) {
        return;
    }
    result->harmonic_order_count = component_count;
    result->dc_offset = dc;

    /* harmonic amplitudes + Vrms */
    float32_t amp_sq_sum = 0.0f;
    for (uint8_t i = 0U; i < component_count; i++) {
        float32_t amp = sqrtf(I[i] * I[i] + Q[i] * Q[i]);
        uint8_t order = order_list[i];
        result->harmonic_orders[i] = order;
        result->harmonic_amps[i] = amp;
        // if (order <= VOL_HARMONIC_ORDER_MAX) {
        //     result->harmonic_amps[order] = amp;
        // }
        amp_sq_sum += amp * amp;
    }
    result->vrms = sqrtf(0.5f * amp_sq_sum);

    /* reconstruct one full cycle at WAVEFORM_SIZE points, find Vpp */
    float32_t max_v = -1e9f;
    float32_t min_v =  1e9f;

    uint16_t oversample = 16;
    for (uint16_t k = 0U; k < oversample * WAVEFORM_SIZE; k++) {
        float32_t x = dc;
        float32_t theta = 2.0f * PI * (float32_t)k / (float32_t)oversample / (float32_t)WAVEFORM_SIZE;

        for (uint8_t i = 0U; i < component_count; i++) {
            float32_t phase = theta * (float32_t)order_list[i];
            x += I[i] * arm_cos_f32(phase) + Q[i] * arm_sin_f32(phase);
        }

        if (k % oversample == oversample - 1)
            result->waveform[k / oversample] = x;

        if (x > max_v) max_v = x;
        if (x < min_v) min_v = x;
    }

    result->vpp = max_v - min_v;
}
