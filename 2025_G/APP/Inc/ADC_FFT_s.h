#ifndef __ADC_FFT_S_H__
#define __ADC_FFT_S_H__

#include <stdint.h>
#include "arm_math.h"
#include <stdbool.h>
#include "app_main.h"

#define FS                      1000000.0f

#define ADC_BUFFER_SIZE         1024
#define DAC_BUFFER_SIZE         1024

#define N_FFT                   ADC_BUFFER_SIZE
#define N_BINS                  (N_FFT/2)

#define BIN_HZ                  (FS/N_FFT)
#define GUARD_RATIO             1e-6f

#define dac_data  (work.dac)
#define sys_data  (work.sys)

typedef union {
    uint16_t    u16[ADC_BUFFER_SIZE];
    q15_t       q15[ADC_BUFFER_SIZE];
} AdcFftBuffer;
typedef union{
    struct {
        q15_t dac[ADC_BUFFER_SIZE];     /* 参考通道：时域 → 频域 */
        q15_t sys[ADC_BUFFER_SIZE];     /* 系统输出：时域 → 频域 */
    };
    float f32[ADC_BUFFER_SIZE];         /* 后期复用：脉冲响应 h[n] */
}WORKBuffer;

extern AdcFftBuffer adc_buffer;
extern WORKBuffer work;
extern volatile bool g_adc_sample_ready;
extern float freq_response[AD9851_SWEEP_FREQ_COUNT*2+1];
// extern float xy_response[AD9851_SWEEP_FREQ_COUNT*4];

void capture_to_spectra(void);
void compute_freq_response(void);
float bin_freq(uint32_t k);
float freq_response_mag_db(uint32_t k);
float freq_response_phase_deg(uint32_t k);
void rfft_get_bin(const q15_t *X, uint32_t k, float *re, float *im);
void project_tone_response(uint32_t freq_hz, float *xr, float *xi, float *yr, float *yi);

#endif
