#include "calibration_vpp.h"
#include "arm_math_types.h"
#include "dac.h"
#include "fpga.h"
#include "main.h"
#include "opamp.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_opamp.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "voltage.h"
#include <math.h>
#include <stdint.h>
#include "dac_sine_lut.h"

#define SAMPLE_CAL_REF_VPP          0.200f
#define SAMPLE_CAL_DISCARD_COUNT    1U
#define SAMPLE_CAL_FRAME_COUNT      5U
#define SAMPLE_CAL_CAPTURE_TIMEOUT  1000U
#define SAMPLE_CAL_MIN_RAW_VPP      1.0f

static vpp_cal_table_t g_vpp_cal;
static uint8_t g_vpp_cal_enabled = 1U;
static float32_t g_sample_cal_raw_vpp_mean = 0.0f;
static float32_t g_sample_cal_to_volt = 0.0f;

static const float32_t freq_hz_init[VPP_CAL_MAX_POINTS] = {10000.000000f,
    13000.000000f,
    16500.000000f,
    21500.000000f,
    28000.000000f,
    36000.000000f,
    46500.000000f,
    60000.000000f,
    77500.000000f,
    100000.000000f,
    109000.000000f,
    118500.000000f,
    129000.000000f,
    140500.000000f,
    152500.000000f,
    166000.000000f,
    181000.000000f,
    197000.000000f,
    214500.000000f,
    233500.000000f,
    254000.000000f,
    276500.000000f,
    301000.000000f,
    327500.000000f,
    356500.000000f,
    388000.000000f,
    422000.000000f,
    459500.000000f,
    500000.000000f
};
static const float32_t gain_corr_init[VPP_CAL_MAX_POINTS] = {
    1.000000000f,
    1.000695894f,
    1.000695894f,
    1.000695894f,
    1.001392758f,
    1.002789400f,
    1.004189944f,
    1.004189944f,
    1.007002801f,
    1.010541110f,
    1.011963406f,
    1.015536723f,
    1.019858156f,
    1.024216524f,
    1.027877055f,
    1.034532374f,
    1.042028986f,
    1.048869439f,
    1.056576046f,
    1.063609467f,
    1.071535022f,
    1.078769693f,
    1.089393939f,
    1.096036586f,
    1.106153846f,
    1.114728682f,
    1.121684868f,
    1.132283464f,
    1.142176330f
};

void vpp_cal_table_init(void){
    g_vpp_cal.count = VPP_CAL_MAX_POINTS;
    g_vpp_cal_enabled = 1U;
    for(uint8_t i = 0 ;i<VPP_CAL_MAX_POINTS;i++){
        g_vpp_cal.freq_hz[i]=freq_hz_init[i];
        g_vpp_cal.gain_corr[i]=gain_corr_init[i];
    }
}

void vpp_cal_set_enabled(uint8_t enabled)
{
    g_vpp_cal_enabled = enabled ? 1U : 0U;
}

uint8_t vpp_cal_is_enabled(void)
{
    return g_vpp_cal_enabled;
}

float32_t cal_corr(float32_t f){
    if (!g_vpp_cal_enabled) {
        return 1.0f;
    }

    if (g_vpp_cal.count == 0U || f <= 0.0f) {
        return 1.0f;
    }

    if (g_vpp_cal.count == 1U || f <= g_vpp_cal.freq_hz[0]) {
        return g_vpp_cal.gain_corr[0];
    }

    uint8_t last = (uint8_t)(g_vpp_cal.count - 1U);
    if (f >= g_vpp_cal.freq_hz[last]) {
        return g_vpp_cal.gain_corr[last];
    }

    float32_t x = logf(f);
    for (uint8_t i = 0U; i < last; i++) {
        float32_t f0 = g_vpp_cal.freq_hz[i];
        float32_t f1 = g_vpp_cal.freq_hz[i + 1U];

        if (f >= f0 && f <= f1) {
            float32_t x0 = logf(f0);
            float32_t x1 = logf(f1);
            float32_t y0 = g_vpp_cal.gain_corr[i];
            float32_t y1 = g_vpp_cal.gain_corr[i + 1U];
            float32_t dx = x1 - x0;

            if (fabsf(dx) < 1.0e-12f) {
                return y0;
            }

            float32_t t = (x - x0) / dx;
            return y0 + t * (y1 - y0);
        }
    }

    return g_vpp_cal.gain_corr[last];
}

static uint8_t get_max_min(const int16_t *buffer, uint32_t len,
                           int16_t *max_out, int16_t *min_out)
{
    if (buffer == NULL || max_out == NULL || min_out == NULL || len == 0U) {
        return 0U;
    }

    int16_t max_v = buffer[0];
    int16_t min_v = buffer[0];

    for (uint32_t i = 1U; i < len; i++) {
        int16_t value = buffer[i];

        if (value > max_v) {
            max_v = value;
        }
        if (value < min_v) {
            min_v = value;
        }
    }

    *max_out = max_v;
    *min_out = min_v;
    return 1U;
}

static uint8_t fpga_capture_frame(void)
{
    uint32_t start_tick = HAL_GetTick();

    fpga_clear_data_ready();
    fpga_start_capture();

    while (!fpga_is_data_ready()) {
        if ((HAL_GetTick() - start_tick) > SAMPLE_CAL_CAPTURE_TIMEOUT) {
            return 0U;
        }
    }

    return fpga_receive();
}

void calibration_start(void){
    voltage_set_sample_to_volt(1.0);
   HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_Stop_DMA(&hdac2, DAC_CHANNEL_1);

    __HAL_DAC_CLEAR_FLAG(&hdac2, DAC_FLAG_DMAUDR1);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);

    hdac3.ErrorCode = HAL_DAC_ERROR_NONE;


    // HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    if (
        // HAL_DAC_Start(&hdac3, DAC_CHANNEL_1)
        HAL_DAC_Start_DMA(&hdac2,
                          DAC_CHANNEL_1,
                          (uint32_t *)dac3_sine_10k_200mvpp_400pt_lut,
                          DAC_SINE_10K_400_POINTS,
                          DAC_ALIGN_12B_R) 
                          != HAL_OK
                        ) {
        Error_Handler();
        return;
    }
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        Error_Handler();
        return;
    }



    float32_t raw_vpp_sum = 0.0f;
    uint8_t valid_count = 0U;

    for (uint8_t i = 0U; i < SAMPLE_CAL_DISCARD_COUNT + SAMPLE_CAL_FRAME_COUNT; i++) {
        if (!fpga_capture_frame()) {
            Error_Handler();
            return;
        }

        if (i < SAMPLE_CAL_DISCARD_COUNT) {
            continue;
        }

        int16_t max_v = 0;
        int16_t min_v = 0;
        if (!get_max_min(fpga_get_buffer(), FPGA_SAMPLE_COUNT, &max_v, &min_v)) {
            Error_Handler();
            return;
        }

        float32_t raw_vpp = (float32_t)((int32_t)max_v - (int32_t)min_v);
        if (raw_vpp > SAMPLE_CAL_MIN_RAW_VPP) {
            raw_vpp_sum += raw_vpp;
            valid_count++;
        }

        HAL_Delay(20U);
    }

    if (valid_count == 0U) {
        Error_Handler();
        return;
    }

    g_sample_cal_raw_vpp_mean = raw_vpp_sum / (float32_t)valid_count;
    g_sample_cal_to_volt = SAMPLE_CAL_REF_VPP / g_sample_cal_raw_vpp_mean;
    voltage_set_sample_to_volt(g_sample_cal_to_volt);

    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_Stop_DMA(&hdac2, DAC_CHANNEL_1);
}

