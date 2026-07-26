#include "ADC_FFT_s.h"
#include "hanning_window_q15.h"
#include "main.h"
#include "arm_math_types.h"
#include "dsp/transform_functions.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// static uint16_t g_dac_buffer[DAC_BUFFER_SIZE];

AdcFftBuffer adc_buffer;

WORKBuffer work;

static q15_t rfft_scratch[RFFT_FULL_Q15_SIZE];

SweepFreqState sweepfreqstate;      //状态机


/* H[k]，re/im 交错，bin 0..N_BINS 共 4097 个 */
float freq_response[ADC_BUFFER_SIZE+2];
float freq_response_accum[FREQ_ACC_COUNT][N_BINS + 1];
float xy_response_buffer[2*(ADC_BUFFER_SIZE+2)];
float coherence_response[N_BINS + 1];
// float xy_response[SWEEP_COUNT][2*(ADC_BUFFER_SIZE+2)];
// float xy_response[AD9851_SWEEP_FREQ_COUNT*4];

volatile bool g_adc_sample_ready = false;

static inline float q15_to_f(q15_t v) { return (float)v * (1.0f / 32768.0f); }

static inline q15_t f_to_q15_sat(float v)
{
    if (v > 0.999969482f) {
        return 32767;
    }
    if (v < -1.0f) {
        return -32768;
    }
    return (q15_t)(v * 32768.0f);
}

static inline float adc_low_byte_to_float(uint16_t sample)
{
    return ((float)(sample & 0xFFU)) * (1.0f / 256.0f);
}

static inline float adc_high_byte_to_float(uint16_t sample)
{
    return ((float)((sample >> 8) & 0xFFU)) * (1.0f / 256.0f);
}

/* CMSIS Q15 RFFT writes a full complex spectrum; keep bins 0..N/2. */
void rfft_get_bin(const q15_t *X, uint32_t k, float *re, float *im)
{
    *re = q15_to_f(X[2 * k]);
    *im = q15_to_f(X[2 * k + 1]);
}


void capture_to_spectra(void)
{
    /* 假设双 ADC 8-bit 打包：低字节 = 参考(DAC) 通道，高字节 = 系统输出 */
    float dac_sum = 0.0f;
    float sys_sum = 0.0f;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        float dac = (adc_buffer.u16[i] & 0xFF) * (1.0f / 256.0f);
        float sys = (adc_buffer.u16[i] >> 8)   * (1.0f / 256.0f);
        dac_sum += dac;
        sys_sum += sys;
    }

    const float dac_mean = dac_sum / (float)ADC_BUFFER_SIZE;
    const float sys_mean = sys_sum / (float)ADC_BUFFER_SIZE;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        float dac = (adc_buffer.u16[i] & 0xFF) * (1.0f / 256.0f) - dac_mean;
        float sys = (adc_buffer.u16[i] >> 8)   * (1.0f / 256.0f) - sys_mean;
        dac_data[i] = (q15_t)(dac * 32768.0f);
        sys_data[i] = (q15_t)(sys * 32768.0f);
    }
    arm_rfft_instance_q15 fft;
    if (arm_rfft_init_q15(&fft, N_FFT, 0, 1) != ARM_MATH_SUCCESS) {
        Error_Handler();   /* 需要 8192 点 Q15 RFFT 表（默认启用） */
    }

    arm_rfft_q15(&fft, dac_data, rfft_scratch);
    memcpy(dac_data, rfft_scratch, RFFT_POSITIVE_Q15_SIZE * sizeof(q15_t));

    arm_rfft_q15(&fft, sys_data, rfft_scratch);
    memcpy(sys_data, rfft_scratch, RFFT_POSITIVE_Q15_SIZE * sizeof(q15_t));
}

void project_tone_response(uint32_t freq_hz, float *xr, float *xi, float *yr, float *yi)
{
    float dac_sum = 0.0f;
    float sys_sum = 0.0f;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        dac_sum += adc_low_byte_to_float(adc_buffer.u16[i]);
        sys_sum += adc_high_byte_to_float(adc_buffer.u16[i]);
    }

    const float dac_mean = dac_sum / (float)ADC_BUFFER_SIZE;
    const float sys_mean = sys_sum / (float)ADC_BUFFER_SIZE;

    // 原逻辑：先 FFT，再从固定 bin 里取值
    // capture_to_spectra();

    const float w = 6.28318530717958647692f * ((float)freq_hz / FS);
    const float cw = cosf(w);
    const float sw = sinf(w);
    const float scale = 65536.0f / (float)HANNING_WINDOW_Q15_SUM_1024;

    float c = 1.0f;
    float s = 0.0f;
    float dac_re = 0.0f;
    float dac_im = 0.0f;
    float sys_re = 0.0f;
    float sys_im = 0.0f;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        q15_t wq15 = hanning_window_q15_1024[i];
        float_t window = wq15/32768.0f; 

        const float dac = adc_low_byte_to_float(adc_buffer.u16[i]) - dac_mean;
        const float sys = adc_high_byte_to_float(adc_buffer.u16[i]) - sys_mean;

        dac_re += dac * c * window;
        dac_im -= dac * s * window;
        sys_re += sys * c * window;
        sys_im -= sys * s * window;

        const float c_next = c * cw - s * sw;
        s = s * cw + c * sw;
        c = c_next;
    }

    *xr = dac_re * scale;
    *xi = dac_im * scale;
    *yr = sys_re * scale;
    *yi = sys_im * scale;
}

/* ---------------- 步骤 3：复除法 H = Y / X ---------------- */

void compute_freq_response(void)
{
    /* 找参考谱峰值，作为相对阈值（带外 bin 的 X≈0，必须排除） */
    float x_max2 = 0.0f;
    for (uint32_t k = 0; k <= N_BINS; k++) {
        float xr, xi;
        rfft_get_bin(dac_data, k, &xr, &xi);
        float m2 = xr * xr + xi * xi;
        if (m2 > x_max2) x_max2 = m2;
    }
    const float x_guard2 = x_max2 * GUARD_RATIO;

    for (uint32_t k = 0; k <= N_BINS; k++) {
        float xr, xi, yr, yi;
        rfft_get_bin(dac_data, k, &xr, &xi);
        rfft_get_bin(sys_data, k, &yr, &yi);

        float d = xr * xr + xi * xi;
        // xy_response_buffer[4*k]=xr;
        // xy_response_buffer[4*k+1]=xi;
        // xy_response_buffer[4*k+2]=yr;
        // xy_response_buffer[4*k+3]=yi;
        if (d < x_guard2) {
            freq_response[2 * k]     = 0.0f;   /* 无效 bin */
            freq_response[2 * k + 1] = 0.0f;
            continue;
        }

        /* (yr + j·yi)/(xr + j·xi)；FFT 缩放比值相消，绝不能再除 N */
        freq_response[2 * k]     = (yr * xr + yi * xi) / d;
        freq_response[2 * k + 1] = (yi * xr - yr * xi) / d;
    }
}

/* ---------------- 结果读取 ---------------- */

void freq_response_accum_reset(void)
{
    memset(freq_response_accum, 0, sizeof(freq_response_accum));
}

void freq_response_accum_add(void)
{
    for (uint32_t k = 0; k <= N_BINS; k++) {
        float xr, xi, yr, yi;
        rfft_get_bin(dac_data, k, &xr, &xi);
        rfft_get_bin(sys_data, k, &yr, &yi);
        if(sweepfreqstate.count == 0){
            xy_response_buffer[4*k]=xr;
            xy_response_buffer[4*k+1]=xi;
            xy_response_buffer[4*k+2]=yr;
            xy_response_buffer[4*k+3]=yi;
        }

        freq_response_accum[FREQ_ACC_RE][k] += yr * xr + yi * xi;
        freq_response_accum[FREQ_ACC_IM][k] += yi * xr - yr * xi;
        freq_response_accum[FREQ_ACC_XX][k] += xr * xr + xi * xi;
        freq_response_accum[FREQ_ACC_YY][k] += yr * yr + yi * yi;
    }
}

void compute_freq_response_from_accum(void)
{
    float den_max = 0.0f;
    for (uint32_t k = 0; k <= N_BINS; k++) {
        float d = freq_response_accum[FREQ_ACC_XX][k];
        if (d > den_max) {
            den_max = d;
        }
    }

    const float den_guard = den_max * GUARD_RATIO;
    for (uint32_t k = 0; k <= N_BINS; k++) {
        const float d = freq_response_accum[FREQ_ACC_XX][k];
        const float yy = freq_response_accum[FREQ_ACC_YY][k];
        const float syx_re = freq_response_accum[FREQ_ACC_RE][k];
        const float syx_im = freq_response_accum[FREQ_ACC_IM][k];
        if (d <= 0.0f || d < den_guard) {
            freq_response[2 * k] = 0.0f;
            freq_response[2 * k + 1] = 0.0f;
            coherence_response[k] = 0.0f;
            continue;
        }

        freq_response[2 * k] = syx_re / d;
        freq_response[2 * k + 1] = syx_im / d;

        float coherence = 0.0f;
        if (yy > 0.0f) {
            coherence = (syx_re * syx_re + syx_im * syx_im) / (d * yy);
            if (coherence > 1.0f) {
                coherence = 1.0f;
            }
        }
        coherence_response[k] = coherence;
    }
}

float bin_freq(uint32_t k) { return k * BIN_HZ; }

float freq_response_mag_db(uint32_t k)
{
    float re = freq_response[2 * k], im = freq_response[2 * k + 1];
    return 10.0f * log10f(re * re + im * im + 1e-12f);   /* 20·log10|H| */
}

float freq_response_phase_deg(uint32_t k)
{
    return atan2f(freq_response[2 * k + 1], freq_response[2 * k])
           * (180.0f / 3.14159265358979f);
}
