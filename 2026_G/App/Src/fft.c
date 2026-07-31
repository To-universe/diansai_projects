#include "fft.h"
#include "app_adc.h"
#include "arm_math.h"
#include "arm_math_types.h"
#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "dsp/transform_functions.h"
#include "hanning_window_q15.h"
#include <math.h>
#include <string.h>

static arm_rfft_instance_q15 fft_instance;
static q15_t   fft_work[FFT_SIZE * 2U];    /* scratch: DC-removed, windowed, then RFFT output */
static float32_t magnitude[FFT_SIZE / 2U];

#define PEAK_CANDIDATE_MAX          24U
#define PEAK_MIN_SEPARATION_BIN      8U
#define PEAK_RELATIVE_MIN            0.05f

typedef struct {
    uint16_t bin;
    float32_t freq;
    float32_t amp;
} peak_candidate_t;

static float32_t abs_f32(float32_t x)
{
    return x >= 0.0f ? x : -x;
}

static int32_t round_to_int(float32_t x)
{
    return (int32_t)(x + 0.5f);
}

static float32_t interp_peak_freq(uint16_t bin)
{
    if (bin == 0U || bin >= (FFT_SIZE / 2U - 1U)) {
        return (float32_t)bin * FS / (float32_t)FFT_SIZE;
    }

    float32_t ym1 = magnitude[bin - 1U];
    float32_t y0  = magnitude[bin];
    float32_t yp1 = magnitude[bin + 1U];

    if (ym1 <= 0.0f || y0 <= 0.0f || yp1 <= 0.0f) {
        return (float32_t)bin * FS / (float32_t)FFT_SIZE;
    }

    ym1 = logf(ym1);
    y0  = logf(y0);
    yp1 = logf(yp1);

    float32_t denom = ym1 - 2.0f * y0 + yp1;
    if (fabsf(denom) < 1.0e-12f) {
        return (float32_t)bin * FS / (float32_t)FFT_SIZE;
    }

    float32_t delta = 0.5f * (ym1 - yp1) / denom;
    if (delta > 0.5f) {
        delta = 0.5f;
    } else if (delta < -0.5f) {
        delta = -0.5f;
    }

    return ((float32_t)bin + delta) * FS / (float32_t)FFT_SIZE;
}

static uint16_t bin_distance(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void sort_candidates_by_amp(peak_candidate_t *candidates, uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++) {
        for (uint8_t j = i + 1U; j < count; j++) {
            if (candidates[j].amp > candidates[i].amp) {
                peak_candidate_t tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }
}

static void sort_result_by_bin(fft_result_t *r)
{
    for (uint8_t i = 0U; i < r->peak_count; i++) {
        for (uint8_t j = i + 1U; j < r->peak_count; j++) {
            if (r->peak_bin[j] < r->peak_bin[i]) {
                uint16_t bin = r->peak_bin[i];
                float32_t freq = r->peak_interp_freq[i];

                r->peak_bin[i] = r->peak_bin[j];
                r->peak_interp_freq[i] = r->peak_interp_freq[j];

                r->peak_bin[j] = bin;
                r->peak_interp_freq[j] = freq;
            }
        }
    }
}

static void find_peaks(float32_t noise_lim, fft_result_t *r)
{
    peak_candidate_t candidates[PEAK_CANDIDATE_MAX] = {0};
    uint8_t candidate_count = 0U;

    r->peak_count = 0U;

    for (uint16_t i = BIN_MIN; i <= BIN_MAX; i++) {
        if (magnitude[i] > magnitude[i - 1U] &&
            magnitude[i] > magnitude[i + 1U] &&
            magnitude[i] > noise_lim) {

            uint8_t pos = candidate_count;
            for (uint8_t k = 0U; k < candidate_count; k++) {
                if (bin_distance(i, candidates[k].bin) <= PEAK_MIN_SEPARATION_BIN) {
                    if (magnitude[i] <= candidates[k].amp) {
                        pos = PEAK_CANDIDATE_MAX;
                    } else {
                        pos = k;
                    }
                    break;
                }
            }

            if (pos == PEAK_CANDIDATE_MAX) {
                continue;
            }

            if (pos == candidate_count) {
                if (candidate_count < PEAK_CANDIDATE_MAX) {
                    candidate_count++;
                } else {
                    uint8_t weakest = 0U;
                    for (uint8_t k = 1U; k < PEAK_CANDIDATE_MAX; k++) {
                        if (candidates[k].amp < candidates[weakest].amp) {
                            weakest = k;
                        }
                    }

                    if (magnitude[i] <= candidates[weakest].amp) {
                        continue;
                    }
                    pos = weakest;
                }
            }

            candidates[pos].bin = i;
            candidates[pos].freq = interp_peak_freq(i);
            candidates[pos].amp = magnitude[i];
        }
    }

    sort_candidates_by_amp(candidates, candidate_count);

    float32_t accept_lim = noise_lim;
    if (candidate_count > 0U) {
        float32_t relative_lim = candidates[0].amp * PEAK_RELATIVE_MIN;
        if (relative_lim > accept_lim) {
            accept_lim = relative_lim;
        }
    }

    for (uint8_t i = 0U; i < candidate_count && r->peak_count < PEAK_MAX_COUNT; i++) {
        if (candidates[i].amp < accept_lim) {
            continue;
        }

        uint8_t idx = r->peak_count;
        r->peak_bin[idx] = candidates[i].bin;
        r->peak_interp_freq[idx] = candidates[i].freq;
        r->peak_count++;
    }

    sort_result_by_bin(r);
}

static void fit_fundamental(fft_result_t *r)
{
    if (r->peak_count == 0U) {
        r->fitted_f0   = 0.0f;
        r->measured_f0 = 0.0f;
        return;
    }

    int32_t g0 = round_to_int(r->peak_interp_freq[0] / FREQ_GRID_HZ);
    float32_t best_score = 1e12f;
    int32_t best_g = 0;
    int32_t g_start = g0 - FUND_SEARCH_RADIUS;
    int32_t g_end   = g0 + FUND_SEARCH_RADIUS;

    if (g_start < G_MIN) g_start = G_MIN;
    if (g_end   > G_MAX) g_end   = G_MAX;

    for (int32_t g = g_start; g <= g_end; g++) {
        float32_t f0 = (float32_t)g * FREQ_GRID_HZ;
        float32_t score = abs_f32(r->peak_interp_freq[0] - f0) * 2.0f;

        for (uint8_t i = 1U; i < r->peak_count; i++) {
            int32_t n = round_to_int(r->peak_interp_freq[i] / f0);
            if (n < 2 || n > HARMONIC_ORDER_MAX) {
                score += 1e12f;
                continue;
            }
            float32_t expected = (float32_t)n * f0;
            score += abs_f32(r->peak_interp_freq[i] - expected);
        }

        if (score < best_score) {
            best_score = score;
            best_g = g;
        }
    }

    r->fitted_f0 = (float32_t)best_g * FREQ_GRID_HZ;
    r->peak_harmonic_order[0] = 1U;
    r->fitted_peak_freq[0] = r->fitted_f0;

    for (uint8_t i = 1U; i < r->peak_count; i++) {
        int32_t n = round_to_int(r->peak_interp_freq[i] / r->fitted_f0);
        r->peak_harmonic_order[i] = (uint8_t)n;
        r->fitted_peak_freq[i] = (float32_t)n * r->fitted_f0;
    }

    float32_t weighted_f0_sum = 0.0f;
    float32_t weight_sum = 0.0f;
    for (uint8_t i = 0U; i < r->peak_count; i++) {
        uint8_t order = r->peak_harmonic_order[i];
        if (order == 0U) {
            continue;
        }
        float32_t weight = magnitude[r->peak_bin[i]];
        weighted_f0_sum += weight * r->peak_interp_freq[i] / (float32_t)order;
        weight_sum += weight;
    }

    r->measured_f0 = r->fitted_f0;
    if (weight_sum > 0.0f) {
        r->measured_f0 = weighted_f0_sum / weight_sum;
    }
}

void fft_init(void)
{
    arm_rfft_init_4096_q15(&fft_instance, 0U, 1U);
}

void fft_compute(const int16_t *data, fft_result_t *result)
{
    memset(result, 0, sizeof(fft_result_t));

    /* DC removal */
    q15_t mean;
    arm_mean_q15(data, FFT_SIZE, &mean);
    arm_offset_q15(data, -mean, fft_work, FFT_SIZE);

    /* windowing in place */
    arm_mult_q15(fft_work, hanning_window_q15_4096, fft_work, FFT_SIZE);

    /* forward RFFT (output overwrites fft_work) */
    arm_rfft_q15(&fft_instance, fft_work, fft_work);

    /* magnitude spectrum */
    result->magnitude = magnitude;
    result->fft_output = fft_work;
    magnitude[0] = (float32_t)fft_work[0];       /* DC bin – real only */
    for (uint16_t i = 1U; i < FFT_SIZE / 2U; i++) {
        float32_t real = (float32_t)fft_work[2U * i];
        float32_t imag = (float32_t)fft_work[2U * i + 1U];
        magnitude[i] = sqrtf(real * real + imag * imag);
    }

    /* noise floor */
    float32_t mag_mean;
    arm_mean_f32(magnitude, FFT_SIZE / 2U, &mag_mean);
    float32_t noise_lim = mag_mean * 4.0f;
    result->noise_floor = noise_lim;

    /* peak detection */
    find_peaks(noise_lim, result);

    /* fundamental-frequency fitting */
    fit_fundamental(result);
}
