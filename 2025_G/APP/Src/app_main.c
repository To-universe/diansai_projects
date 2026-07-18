#include "app_main.h"
#include "AD9851.h"
#include "ADC.h"
#include "ADC_FFT_s.h"
#include "../../Core/Inc/adc.h"
#include "cmsis_gcc.h"
#include "dsp/transform_functions.h"
#include "stm32_hal_legacy.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>

static void uart_send_payload(const uint8_t magic[4], const void *payload, uint32_t payload_size, uint32_t timeout)
{
    if (HAL_UART_Transmit(&huart5, magic, 4U, 1000) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UART_Transmit(&huart5, (const uint8_t *)&payload_size, sizeof(payload_size), 1000) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UART_Transmit(&huart5, (const uint8_t *)payload, payload_size, timeout) != HAL_OK) {
        Error_Handler();
    }
}

static void uart_send_adc_buffer(void)
{
    static const uint8_t magic[4] = {'A', 'D', 'C', 'T'};
    uart_send_payload(magic, adc_buffer.u16, sizeof(adc_buffer.u16), 50000);
}

static void uart_send_raw_frame(const uint8_t magic[4], const void *payload, uint32_t payload_size)
{
    if (HAL_UART_Transmit(&huart5, magic, 4, 5000) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UART_Transmit(&huart5, (uint8_t *)payload, payload_size, 5000) != HAL_OK) {
        Error_Handler();
    }
}

static void uart_send_mag_response(uint32_t k0, uint32_t k1)
{
    static const uint8_t magic[4] = {'F', 'R', 'S', 'P'};
    uint32_t payload_size = (k1 - k0 + 1U) * sizeof(float);

    if (HAL_UART_Transmit(&huart5, magic, sizeof(magic), 1000) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UART_Transmit(&huart5, (const uint8_t *)&payload_size, sizeof(payload_size), 1000) != HAL_OK) {
        Error_Handler();
    }

    for (uint32_t k = k0; k <= k1; k++) {
        float mag_db;
        if (freq_response[2 * k] == 0.0f && freq_response[2 * k + 1] == 0.0f) {
            mag_db = -120.0f;
        } else {
            mag_db = freq_response_mag_db(k);
        }

        if (HAL_UART_Transmit(&huart5, (const uint8_t *)&mag_db, sizeof(mag_db), 1000) != HAL_OK) {
            Error_Handler();
        }
    }
}

void app_main(void)
{
    AD9851_RESET();
    AD9851_SweepFreq_calc(1000, 10000);
    // HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer.u16, ADC_BUFFER_SIZE);
    AD9851_Sweepstart(SweepFreq_value, AD9851_SWEEP_FREQ_COUNT);
    ADC_wait_stable();
    // while (!g_adc_sample_ready) {
    //     __WFI();
    // }

    // HAL_ADCEx_MultiModeStop_DMA(&hadc1);

    // capture_to_spectra();
    // compute_freq_response();

    // 原逻辑里用于限定扫频对应的 FFT bin 范围
    // uint32_t k0 = (uint32_t)ceilf(1000 / BIN_HZ);
    // uint32_t k1 = (uint32_t)floorf(10000 / BIN_HZ);
    while (1) {
        if(adc_wait_flag){
            now_tick = HAL_GetTick();
            if(now_tick-stable_tick >=1){
                ADC_sample_start();
                adc_wait_flag=0;
            }
        }

        if(adc_ready){
            adc_ready = 0;
            if(ad9851_sweep.index >= AD9851_SWEEP_FREQ_COUNT){
                continue;
            }
            const uint32_t freq_hz = ad9851_sweep.freq_table[ad9851_sweep.index];
            float xr, xi, yr, yi;
            // 原逻辑：先做 FFT，再通过 rfft_get_bin 取指定频点
            // capture_to_spectra();
            // uint32_t bin_sel = (uint32_t)((((uint64_t)freq_hz * (uint64_t)N_FFT) + ((uint64_t)FS / 2U)) / (uint64_t)FS);
            // if (bin_sel < 1U) {
            //     bin_sel = 1U;
            // } else if (bin_sel > N_BINS) {
            //     bin_sel = N_BINS;
            // }
            // rfft_get_bin(dac_data, bin_sel, &xr, &xi);
            // rfft_get_bin(sys_data, bin_sel, &yr, &yi);

            project_tone_response(freq_hz, &xr, &xi, &yr, &yi);

            float x_max2 = xr * xr + xi * xi;
            const float x_guard2 = x_max2 * GUARD_RATIO;
            // 原逻辑：把 X/Y 的实部虚部打包发给 Python
            // xy_response[4*ad9851_sweep.index] = xr;
            // xy_response[4*ad9851_sweep.index+1] = xi;
            // xy_response[4*ad9851_sweep.index+2] = yr;
            // xy_response[4*ad9851_sweep.index+3] = yi;
            uint8_t xy_head[4] = {'X','Y','R','T'};
            HAL_UART_Transmit(&huart5, xy_head, 4, 5000);
            HAL_UART_Transmit(&huart5, (const uint8_t *)&xr, sizeof(xr), 5000);
            HAL_UART_Transmit(&huart5, (const uint8_t *)&xi, sizeof(xi), 5000);
            HAL_UART_Transmit(&huart5, (const uint8_t *)&yr, sizeof(yr), 5000);
            HAL_UART_Transmit(&huart5, (const uint8_t *)&yi, sizeof(yi), 5000);
            float_t d = xr * xr + xi * xi;
            if(d<x_guard2){
                freq_response[2*ad9851_sweep.index]=0.0f;
                freq_response[2*ad9851_sweep.index+1]=0.0f;
            }else{
                freq_response[2*ad9851_sweep.index]=(yr * xr + yi * xi) / d;
                freq_response[2*ad9851_sweep.index+1]=(yi * xr - yr * xi) / d;
            }

        }

        if(ad9851_sweep.index == AD9851_SWEEP_FREQ_COUNT){
            static const uint8_t freq_head[4] = {'F', 'S', 'R', 'T'};
            // static const uint8_t xy_head[4] = {'X', 'Y', 'R', 'T'};
            uart_send_raw_frame(freq_head, freq_response, sizeof(freq_response));
            // uart_send_raw_frame(xy_head, xy_response, sizeof(xy_response));
            HAL_Delay(1000);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        AD9851_SweepCallback();
        if (ad9851_sweep.isrunningflag) {
            ADC_wait_stable();
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        // g_adc_sample_ready = true;
        HAL_TIM_Base_Stop(&htim2);
        HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        adc_sampling=0;
        adc_ready=1;
    }
}
