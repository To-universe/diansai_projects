#include "app_main.h"

#include "arm_math.h"
#include "dsp/complex_math_functions.h"
#include "dsp/matrix_functions.h"
#include "dsp/support_functions.h"
#include "dsp/transform_functions.h"
#include "dsp/window_functions.h"
#include "sine_generator.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_dac.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "arm_math.h"

#define FS                1328125.0f   /* DAC/ADC 采样率 [Hz] */

#define ADC_BUFFER_SIZE   8192         /* 每通道采样点数 */
#define DAC_BUFFER_SIZE   1024

#define N_FFT             ADC_BUFFER_SIZE
#define N_BINS            (N_FFT / 2)  /* 4096 个复数 bin */

#define CHIRP_F_START     1000.0f
#define CHIRP_F_STOP      75000.0f
/* chirp 长度 = DAC 循环缓冲长度：激励周期化，8192 窗内 4 个整周期，相干采样免加窗 */
#define CHIRP_LENGTH      (DAC_BUFFER_SIZE * 2)

#define BIN_HZ            (FS / N_FFT) /* 162.14 Hz/bin */
#define GUARD_RATIO       1e-6f        /* 参考谱阈值：峰值以下 60 dB 判无效 */

/* ================= RAM 布局（G474VE 共 128 KB） =================
 * g_dac_buffer    2048 x u16      4 KB
 * adc_buffer      union          16 KB   原始采样 → FFT 暂存
 * work            union          32 KB   dac_data + sys_data → 脉冲响应
 * freq_response   8192 x float   32 KB   H[k] (re,im 交错)
 * 静态合计 84 KB，加栈/HAL 全局变量，远小于 128 KB
 * ============================================================= */

static uint16_t g_dac_buffer[DAC_BUFFER_SIZE * 2];

static union {
    uint16_t u16[ADC_BUFFER_SIZE];      /* 双 ADC 打包原始数据 */
    q15_t    q15[ADC_BUFFER_SIZE];      /* RFFT 输出暂存 */
} adc_buffer;

static union {
    struct {
        q15_t dac[ADC_BUFFER_SIZE];     /* 参考通道：时域 → 频域 */
        q15_t sys[ADC_BUFFER_SIZE];     /* 系统输出：时域 → 频域 */
    };
    float f32[ADC_BUFFER_SIZE];         /* 后期复用：脉冲响应 h[n] */
} work;
#define dac_data  (work.dac)
#define sys_data  (work.sys)

/* H[k]，re/im 交错，bin 0..N_BINS 共 4097 个 */
static float freq_response[ADC_BUFFER_SIZE + 1];

SineGenerator sg;
volatile bool g_adc_sample_ready = false;

/* ---------------- 工具函数 ---------------- */

static inline float q15_to_f(q15_t v) { return (float)v * (1.0f / 32768.0f); }

/* 解包 CMSIS RFFT 格式：[Re0, Re(N/2), Re1, Im1, ...] */
static void rfft_get_bin(const q15_t *X, uint32_t k, float *re, float *im)
{
    if (k == 0)           { *re = q15_to_f(X[0]);     *im = 0.0f; }
    else if (k == N_BINS) { *re = q15_to_f(X[1]);     *im = 0.0f; }
    else                  { *re = q15_to_f(X[2 * k]); *im = q15_to_f(X[2 * k + 1]); }
}

/* ---------------- 步骤 1+2：解包、Q15 化、FFT ---------------- */

static void capture_to_spectra(void)
{
    /* 假设双 ADC 8-bit 打包：低字节 = 参考(DAC) 通道，高字节 = 系统输出 */
    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        float dac = (adc_buffer.u16[i] & 0xFF) * (1.0f / 256.0f);
        float sys = (adc_buffer.u16[i] >> 8)   * (1.0f / 256.0f);
        dac_data[i] = (q15_t)(dac * 32768.0f);
        sys_data[i] = (q15_t)(sys * 32768.0f);
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

/* ---------------- 步骤 3：复除法 H = Y / X ---------------- */

static void compute_freq_response(void)
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

static float bin_freq(uint32_t k) { return k * BIN_HZ; }

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

/* 串口导出 CSV（需链接 -u _printf_float） */
void freq_response_dump_uart(void)
{
    uint32_t k0 = (uint32_t)ceilf(CHIRP_F_START / BIN_HZ);
    uint32_t k1 = (uint32_t)floorf(CHIRP_F_STOP / BIN_HZ);

    printf("freq_Hz,mag_dB,phase_deg\r\n");
    for (uint32_t k = k0; k <= k1; k++) {
        if (freq_response[2 * k] == 0.0f && freq_response[2 * k + 1] == 0.0f)
            continue;   /* 跳过无效 bin（chirp 谱线之间的 bin 也是 0） */
        printf("%.1f,%.2f,%.2f\r\n", bin_freq(k),
               freq_response_mag_db(k), freq_response_phase_deg(k));
    }
}

/* 可选：由 H[k] 求脉冲响应 h[n]。
 * 会破坏 freq_response（复用其内存做 IFFT 输入），务必在导出频率响应之后调用。
 * 返回 N_FFT 个 float（复用 work 缓冲区）。 */
float *freq_response_impulse(void)
{
    static arm_rfft_fast_instance_f32 rfft_f32;
    (void)arm_rfft_fast_init_f32(&rfft_f32, N_FFT);

    freq_response[1] = freq_response[2 * N_BINS];  /* 排回 CMSIS 格式：Nyquist 实部进 slot 1 */
    arm_rfft_fast_f32(&rfft_f32, freq_response, work.f32, 1);  /* 逆变换已含 1/N */
    return work.f32;
}

/* ---------------- 主流程 ---------------- */

void app_main(void)
{
    sine_generator_init(&sg, FS);
    sine_generator_chirp_begin(&sg, CHIRP_F_START, CHIRP_F_STOP, ADC_BUFFER_SIZE);
    sine_generator_fill_buffer(&sg, g_dac_buffer, DAC_BUFFER_SIZE * 2);

    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer.u16, ADC_BUFFER_SIZE);
    HAL_OPAMP_Start(&hopamp1);
    HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1, (uint32_t *)g_dac_buffer,
                      DAC_BUFFER_SIZE * 2, DAC_ALIGN_8B_R);
    HAL_TIM_Base_Start(&htim6);

    while (!g_adc_sample_ready) { __WFI(); }

    HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_1);
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim6);

    capture_to_spectra();        /* 解包 + Q15 + 两次 RFFT */
    compute_freq_response();     /* 复除法 → freq_response */

    uint32_t k0 = (uint32_t)ceilf(CHIRP_F_START / BIN_HZ);
    uint32_t k1 = (uint32_t)floorf(CHIRP_F_STOP / BIN_HZ);

    // printf("freq_Hz,mag_dB,phase_deg\r\n");
    for (uint32_t k = k0; k <= k1; k++) {
        if (freq_response[2 * k] == 0.0f && freq_response[2 * k + 1] == 0.0f)
            continue;   /* 跳过无效 bin（chirp 谱线之间的 bin 也是 0） */
       freq_response[k - k0] = freq_response_mag_db(k);
    }

    // freq_response_dump_uart();   /* 导出幅频/相频 */
    /* float *h = freq_response_impulse();   // 可选：冲击响应 */

    for (;;) { }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) g_adc_sample_ready = true;
}

// #define FS 1328125.

// #define ADC_BUFFER_SIZE 8192
// #define DAC_BUFFER_SIZE 1024

// SineGenerator sg;

// uint16_t g_dac_buffer[DAC_BUFFER_SIZE * 2];

// volatile bool g_adc_sample_ready = false;
// uint16_t adc_buffer[ADC_BUFFER_SIZE];

// uint32_t g_adc_index = 0;

// q15_t dac_data[ADC_BUFFER_SIZE];
// q15_t sys_data[ADC_BUFFER_SIZE];

// q15_t freq_response[ADC_BUFFER_SIZE];

// // float hann_window[ADC_BUFFER_SIZE];

// // q15_t fft_mag[ADC_BUFFER_SIZE / 2 + 1];

// // bool chirp_state

// void app_main(void) {
//     // sine_generator_init(&sg, 664062.5);
//     sine_generator_init(&sg, 1328125);
//     sine_generator_chirp_begin(&sg, 1e3, 75e3, ADC_BUFFER_SIZE);
//     sine_generator_fill_buffer(&sg, g_dac_buffer, DAC_BUFFER_SIZE * 2);

//     // HAL_ADC_Start_DMA(&hadc2, adc_buffer, ADC_BUFFER_SIZE);
//     HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);

//     HAL_OPAMP_Start(&hopamp1);
//     HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1, (uint32_t *)g_dac_buffer, DAC_BUFFER_SIZE * 2, DAC_ALIGN_8B_R);

//     HAL_TIM_Base_Start(&htim6);

//     while (!g_adc_sample_ready);

//     HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_1);

//     /* study input wave
//      * - transform adc input to Q15 format
//      * - perform fft
//      * - find peaks (especially fundamental frequency)
//      * - apply frequency response
//      * - perform ifft
//      * - clip at full cycle (using fundamental frequency)
//      **/

//     // arm_hanning_f32(hann_window, ADC_BUFFER_SIZE);

//     for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++) {
//         float dac = (adc_buffer[i] & 0xFF) / 256.;
//         // float dac = (adc_buffer[i] & 0xFF) / 256. * hann_window[i];
//         dac_data[i] = dac * (1 << 15);

//         float sys = (adc_buffer[i] >> 8) / 256.;
//         // float sys = (adc_buffer[i] >> 8) / 256. * hann_window[i];
//         sys_data[i] = sys * (1 << 15);
//     }
    
//     arm_rfft_instance_q15 fft_instance;
//     arm_rfft_init_q15(&fft_instance, ADC_BUFFER_SIZE, 0, 1);

//     q15_t *fft_tmp = freq_response;

//     arm_rfft_q15(&fft_instance, dac_data, fft_tmp);
//     memcpy(dac_data, fft_tmp, sizeof(dac_data));
//     // arm_cmplx_mag_q15(fft_tmp, dac_data, ADC_BUFFER_SIZE / 2);

//     arm_rfft_q15(&fft_instance, sys_data, fft_tmp);
//     memcpy(sys_data, fft_tmp, sizeof(sys_data));
//     // arm_cmplx_mag_q15(fft_tmp, sys_data, ADC_BUFFER_SIZE / 2);

//     // frequency response requires division
//     // fixed-point arithmetic is bad at this
//     // we do division in floating-point
//     // reusing g_adc_buffer which could fit half
//     // do it twice

//     float *dac_tmp = (float *)&adc_buffer[0];
//     float *sys_tmp = (float *)&adc_buffer[ADC_BUFFER_SIZE / 2];
//     uint32_t tmp_buffer_size = ADC_BUFFER_SIZE / 2;

//     arm_q15_to_float(dac_data, dac_tmp, tmp_buffer_size);
//     arm_q15_to_float(sys_data, sys_tmp, tmp_buffer_size);

//     for (uint32_t i = 0; i < tmp_buffer_size / 2; i++) {
//         float num_real = sys_tmp[2 * i + 0];
//         float num_imag = sys_tmp[2 * i + 1];
//         float den_real = dac_tmp[2 * i + 0];
//         float den_imag = dac_tmp[2 * i + 1];

//         // (a + j b) / (c + j d) = ((ac + bd) + j (bc - ad)) / (c^2 + d^2)
//         float res_den = den_real * den_real + den_imag * den_imag;

//         if (res_den < 1e-12) {
//             dac_tmp[2 * i + 0] = 0;
//             dac_tmp[2 * i + 1] = 0;
//         } else {
//             float res_num_real = (num_real * den_real + num_imag * den_imag) / res_den;
//             float res_num_imag = (num_imag * den_real - num_real * den_imag) / res_den;
//             float res_real = res_num_real / ADC_BUFFER_SIZE;
//             float res_imag = res_num_imag / ADC_BUFFER_SIZE;

//             dac_tmp[2 * i + 0] = res_real;
//             dac_tmp[2 * i + 1] = res_imag;
//         }
//     }

//     // arm_cmplx_mag_f32(dac_tmp, sys_tmp, tmp_buffer_size / 2);

//     arm_float_to_q15(dac_tmp, &freq_response[0], tmp_buffer_size);
//     // arm_cmplx_mag_q15(freq_response, dac_data, ADC_BUFFER_SIZE / 2);

//     // q15_t denominator[ADC_BUFFER_SIZE + 1];
//     // arm_cmplx_mag_squared_q15(dac_fft, denominator, ADC_BUFFER_SIZE + 1);

//     // arm_

//     // q15_t denxd

//     // fft_mag[0] = fft_result[0];
//     // fft_mag[ADC_BUFFER_SIZE / 2] = fft_result[1];
//     // arm_cmplx_mag_q15(&fft_result[2], &fft_mag[1], ADC_BUFFER_SIZE / 2 - 1);

//     // q15_t peak_window[7] = {
//     //     0,
//     //     fft_mag[1],
//     //     fft_mag[2],
//     //     fft_mag[3],
//     //     fft_mag[4],
//     //     fft_mag[5],
//     //     fft_mag[6],
//     // };
//     // uint16_t peak_indices[5];
//     // q15_t peak_magnitudes[5];
//     // uint8_t peak_count = 0;

//     // for (uint32_t i = 7; i < ADC_BUFFER_SIZE / 2 + 1; i++) {
//     //     for (uint32_t j = 0; j < 6; j++) peak_window[j] = peak_window[j + 1];
//     //     peak_window[6] = fft_mag[i];

//     //     if (
//     //         peak_window[0] < peak_window[1] && peak_window[1] < peak_window[2] && peak_window[2] < peak_window[3]
//     //         && peak_window[3] > peak_window[4] && peak_window[4] > peak_window[5] && peak_window[5] > peak_window[6]
//     //     ) {
//     //         peak_indices[peak_count] = i - 3;
//     //         peak_magnitudes[peak_count] = peak_window[3];
//     //         if (++peak_count == 5) break;
//     //     }
//     // }

//     // float fundamental_frequency = (float)peak_indices[0] * FS / (float)ADC_BUFFER_SIZE;

//     // check input's fundamental wave and peaks
    
//     // sine_generator_single_freq_begin(&sg, fundamental_frequency);
//     // sine_generator_fill_buffer(&sg, dac_buffer, DAC_BUFFER_SIZE * 2);

//     // memset(dac_buffer, 0, DAC_BUFFER_SIZE * 2 * 2);

//     // for (uint32_t i = 0; i < DAC_BUFFER_SIZE * 4; i++) {
//     //     if (i % 2 == 0)
//     //         g_dac_buffer[i >> 1] = adc_buffer[g_adc_index] & 0xFF;
//     //     else
//     //         g_dac_buffer[i >> 1] |= (adc_buffer[g_adc_index] & 0xFF) << 8;
//     //     // if (i % 2 == 0)
//     //     //     dac_buffer[i >> 1] = fft_mag[g_adc_index] >> 8;
//     //     // else
//     //     //     dac_buffer[i >> 1] |= fft_mag[g_adc_index] & 0xFF00;
//     //     g_adc_index += 1;
//     //     if (g_adc_index == ADC_BUFFER_SIZE) g_adc_index = 0;
//     // }
// }

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//     g_adc_sample_ready = true;
// }

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    if (sg.chirp_running) sine_generator_fill_buffer(&sg, &g_dac_buffer[0], DAC_BUFFER_SIZE);
    // else {
    //     for (uint32_t i = 0; i < DAC_BUFFER_SIZE * 2; i++) {
    //         if (i % 2 == 0)
    //             dac_buffer[i >> 1] = adc_buffer[g_adc_index] >> 8;
    //         else
    //             dac_buffer[i >> 1] |= adc_buffer[g_adc_index] & 0xFF00;
    //         g_adc_index += 1;
    //         if (g_adc_index == ADC_BUFFER_SIZE) g_adc_index = 0;
    //     }
    // }
    // else {
    //     for (uint32_t i = 0; i < DAC_BUFFER_SIZE * 2; i++) {
    //         if (i % 2 == 0)
    //             g_dac_buffer[i >> 1] = adc_buffer[g_adc_index++] & 0xFF;
    //         else
    //             g_dac_buffer[i >> 1] |= (adc_buffer[g_adc_index++] & 0xFF) << 8;
    //         if (g_adc_index == ADC_BUFFER_SIZE) g_adc_index = 0;
    //     }
    // }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    if (sg.chirp_running) sine_generator_fill_buffer(&sg, &g_dac_buffer[DAC_BUFFER_SIZE], DAC_BUFFER_SIZE);
    // else {
    //     for (uint32_t i = 0; i < DAC_BUFFER_SIZE * 2; i++) {
    //         if (i % 2 == 0)
    //             dac_buffer[DAC_BUFFER_SIZE + (i >> 1)] = fft_mag[g_adc_index] >> 8;
    //         else
    //             dac_buffer[DAC_BUFFER_SIZE + (i >> 1)] |= fft_mag[g_adc_index] & 0xFF00;
    //         g_adc_index += 1;
    //         if (g_adc_index == ADC_BUFFER_SIZE / 2 + 1) g_adc_index = 0;
    //     }
    // }
    // else {
    //     for (uint32_t i = 0; i < DAC_BUFFER_SIZE * 2; i++) {
    //         if (i % 2 == 0)
    //             g_dac_buffer[DAC_BUFFER_SIZE + (i >> 1)] = adc_buffer[g_adc_index++] & 0xFF;
    //         else
    //             g_dac_buffer[DAC_BUFFER_SIZE + (i >> 1)] |= (adc_buffer[g_adc_index++] & 0xFF) << 8;
    //         if (g_adc_index == ADC_BUFFER_SIZE) g_adc_index = 0;
    //     }
    // }
}

