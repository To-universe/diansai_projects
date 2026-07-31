#ifndef __FFT_H__
#define __FFT_H__

#include "arm_math_types.h"
#include "fpga.h"

#define FS Fs_FPGA

#define FFT_SIZE            4096U
#define BIN_MIN             18U
#define BIN_MAX             1027U
#define PEAK_MAX_COUNT      3U
#define FREQ_GRID_HZ        500.0f
#define G_MIN               20
#define G_MAX               1000
#define HARMONIC_ORDER_MAX  (G_MAX / G_MIN)
#define FUND_SEARCH_RADIUS  2

typedef struct {
    float32_t noise_floor;
    uint8_t  peak_count;
    uint16_t peak_bin[PEAK_MAX_COUNT];
    float32_t peak_interp_freq[PEAK_MAX_COUNT];
    float32_t fitted_f0;
    float32_t measured_f0;
    uint8_t  peak_harmonic_order[PEAK_MAX_COUNT];
    float32_t fitted_peak_freq[PEAK_MAX_COUNT];

    /* direct access to the internal spectrum for harmonic_decompose_v2 */
    const float32_t *magnitude;
    const q15_t     *fft_output;
} fft_result_t;

void fft_init(void);
void fft_compute(const int16_t *data, fft_result_t *result);

#endif
