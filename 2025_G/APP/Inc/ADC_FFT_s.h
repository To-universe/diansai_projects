#ifndef __ADC_FFT_S_H__
#define __ADC_FFT_S_H__

#include <stdint.h>
#include "arm_math.h"
#include <stdbool.h>
#include "app_main.h"

#define FS                      100000.0f

#define ADC_BUFFER_SIZE         2048
#define DAC_BUFFER_SIZE         1024

#define N_FFT                   ADC_BUFFER_SIZE
#define N_BINS                  (N_FFT/2)
#define RFFT_FULL_Q15_SIZE      (2 * ADC_BUFFER_SIZE)
#define RFFT_POSITIVE_Q15_SIZE  (ADC_BUFFER_SIZE + 2)

#define BIN_HZ                  (FS/N_FFT)
#define GUARD_RATIO             1e-6f

#define dac_data  (work.dac)
#define sys_data  (work.sys)
#define  SWEEP_COUNT 16
#define FREQ_ACC_RE  0
#define FREQ_ACC_IM  1
#define FREQ_ACC_XX  2
#define FREQ_ACC_YY  3
#define FREQ_ACC_COUNT 4

typedef union {
    uint16_t    u16[ADC_BUFFER_SIZE];
    q15_t       q15[ADC_BUFFER_SIZE];
} AdcFftBuffer;
typedef struct{
    struct {
        q15_t dac[RFFT_POSITIVE_Q15_SIZE];     /* 参考通道：时域 → 正频率复数谱 */
        q15_t sys[RFFT_POSITIVE_Q15_SIZE];     /* 系统输出：时域 → 正频率复数谱 */
    };
}WORKBuffer;
typedef struct{
    uint8_t state;
    uint8_t count;
} SweepFreqState ;
#define SweepFreqStart 0
#define SweepFreqAct 1
#define SweepFreqCalc 2
#define SweepFreqReady 3
#define SweepFreqStop 4

extern AdcFftBuffer adc_buffer;
extern WORKBuffer work;
extern SweepFreqState sweepfreqstate;
extern volatile bool g_adc_sample_ready;
extern float freq_response[ADC_BUFFER_SIZE+2];
extern float freq_response_accum[FREQ_ACC_COUNT][N_BINS + 1];
extern float xy_response_buffer[2*(ADC_BUFFER_SIZE+2)];
extern float coherence_response[N_BINS + 1];


void capture_to_spectra(void);
void compute_freq_response(void);
void freq_response_accum_reset(void);
void freq_response_accum_add(void);
void compute_freq_response_from_accum(void);
float bin_freq(uint32_t k);
float freq_response_mag_db(uint32_t k);
float freq_response_phase_deg(uint32_t k);
void rfft_get_bin(const q15_t *X, uint32_t k, float *re, float *im);
void project_tone_response(uint32_t freq_hz, float *xr, float *xi, float *yr, float *yi);

#endif
