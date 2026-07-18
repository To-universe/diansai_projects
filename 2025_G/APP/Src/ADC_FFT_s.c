#include "ADC_FFT_s.h"
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


/* H[k]，re/im 交错，bin 0..N_BINS 共 4097 个 */
float freq_response[AD9851_SWEEP_FREQ_COUNT*2+1];
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

/* 解包 CMSIS RFFT 格式：[Re0, Re(N/2), Re1, Im1, ...] */
void rfft_get_bin(const q15_t *X, uint32_t k, float *re, float *im)
{
    if (k == 0)           { *re = q15_to_f(X[0]);     *im = 0.0f; }
    else if (k == N_BINS) { *re = q15_to_f(X[1]);     *im = 0.0f; }
    else                  { *re = q15_to_f(X[2 * k]); *im = q15_to_f(X[2 * k + 1]); }
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

    // 原逻辑：直接把 ADC 码值转成 Q15
    // for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    //     float dac = (adc_buffer.u16[i] & 0xFF) * (1.0f / 256.0f);
    //     float sys = (adc_buffer.u16[i] >> 8)   * (1.0f / 256.0f);
    //     dac_data[i] = (q15_t)(dac * 32768.0f);
    //     sys_data[i] = (q15_t)(sys * 32768.0f);
    // }

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        float dac = (adc_buffer.u16[i] & 0xFF) * (1.0f / 256.0f) - dac_mean;
        float sys = (adc_buffer.u16[i] >> 8)   * (1.0f / 256.0f) - sys_mean;
        dac_data[i] = f_to_q15_sat(dac);
        sys_data[i] = f_to_q15_sat(sys);
    }

    arm_rfft_instance_q15 fft;
    if (arm_rfft_init_q15(&fft, N_FFT, 0, 1) != ARM_MATH_SUCCESS) {
        Error_Handler();   /* 需要 8192 点 Q15 RFFT 表（默认启用） */
    }

    q15_t *scratch = adc_buffer.q15;   /* 原始数据已解包，缓冲区复用 */

    arm_rfft_q15(&fft, dac_data, scratch);
    memcpy(dac_data, scratch, sizeof(dac_data));

    arm_rfft_q15(&fft, sys_data, scratch);
    memcpy(sys_data, scratch, sizeof(sys_data));
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
    const float scale = 2.0f / (float)ADC_BUFFER_SIZE;

    float c = 1.0f;
    float s = 0.0f;
    float dac_re = 0.0f;
    float dac_im = 0.0f;
    float sys_re = 0.0f;
    float sys_im = 0.0f;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        const float dac = adc_low_byte_to_float(adc_buffer.u16[i]) - dac_mean;
        const float sys = adc_high_byte_to_float(adc_buffer.u16[i]) - sys_mean;

        dac_re += dac * c;
        dac_im -= dac * s;
        sys_re += sys * c;
        sys_im -= sys * s;

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
