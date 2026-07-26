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
    static const uint32_t sweep_start_hz = 1000U;
    static const uint32_t sweep_stop_hz = 50000U;

    AD9851_RESET();
    sweepfreqstate.state=SweepFreqReady;
    sweepfreqstate.count=0;
    AD9851_SweepFreq_LineCalc(sweep_start_hz, sweep_stop_hz);
    // HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer.u16, ADC_BUFFER_SIZE);
    // AD9851_Sweepstart(SweepFreq_value, AD9851_SWEEP_FREQ_COUNT);
    // HAL_TIM_Base_Start_IT(&htim3);
    // HAL_TIM_Base_Start(&htim2);
    
    // __HAL_TIM_SET_COUNTER(&htim2, 0);
    // HAL_TIM_Base_Start(&htim2);
    
    
    
    
    
    // while (!g_adc_sample_ready) {
    //     __WFI();
    // }
    // g_adc_sample_ready=0;
    // HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    // // uart_send_adc_buffer();
    // HAL_TIM_Base_Stop(&htim2);
    // HAL_TIM_Base_Stop_IT(&htim3);

    // capture_to_spectra();
    // compute_freq_response();

    // 原逻辑里用于限定扫频对应的 FFT bin 范围
    uint32_t k0 = (uint32_t)ceilf((float)sweep_start_hz / BIN_HZ);
    uint32_t k1 = (uint32_t)floorf((float)sweep_stop_hz / BIN_HZ);
    if (k0 > N_BINS) {
        k0 = N_BINS;
    }
    if (k1 > N_BINS) {
        k1 = N_BINS;
    }
    freq_response_accum_reset();
    while (1) {
        if(sweepfreqstate.count<SWEEP_COUNT){
            if(sweepfreqstate.state == SweepFreqReady){
                sweepfreqstate.state = SweepFreqStart;
            }else if(sweepfreqstate.state == SweepFreqStart){
                sweepfreqstate.state = SweepFreqAct;
            }else if(sweepfreqstate.state == SweepFreqAct){
                if(g_adc_sample_ready){
                    sweepfreqstate.state = SweepFreqCalc;
                    g_adc_sample_ready = 0;
                }
            }else if(sweepfreqstate.state == SweepFreqCalc){
                sweepfreqstate.state = SweepFreqReady;
                sweepfreqstate.count++;
            }
        }
        else if(sweepfreqstate.count == SWEEP_COUNT){
            sweepfreqstate.state=SweepFreqStop;
        }

        if(sweepfreqstate.state == SweepFreqStart){
            HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer.u16, ADC_BUFFER_SIZE);
            AD9851_Sweepstart(SweepFreq_value,  AD9851_SWEEP_FREQ_COUNT);
            // AD9851_set_Frequency(30000);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            __HAL_TIM_SET_COUNTER(&htim2,0);
            HAL_TIM_Base_Start_IT(&htim3);
            HAL_TIM_Base_Start(&htim2);
        }else if(sweepfreqstate.state == SweepFreqCalc){
            HAL_TIM_Base_Stop(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);
            HAL_ADCEx_MultiModeStop_DMA(&hadc1);
            if(sweepfreqstate.count == 0){
                uart_send_adc_buffer();
            }
            capture_to_spectra();
            freq_response_accum_add();
        }else if(sweepfreqstate.state == SweepFreqStop){
            compute_freq_response_from_accum();
            static const uint8_t xy_word[4] = {'X', 'Y', 'R', 'T'};
            uart_send_raw_frame(xy_word, xy_response_buffer, sizeof(xy_response_buffer));
            freq_response_filter();
            for(uint32_t i = k0 ;i<=k1;i++){
                float freq_mag_db = freq_response_mag_db(i);
                uint8_t word[4]="FSRT";
                HAL_UART_Transmit(&huart5, word, 4, 1000);
                HAL_UART_Transmit(&huart5, (const uint8_t *)&freq_mag_db, sizeof(freq_mag_db),1000);
            }
            sweepfreqstate.state = SweepFreqReady;
            sweepfreqstate.count = SWEEP_COUNT + 1U;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        AD9851_SweepCallback();
        // if (ad9851_sweep.isrunningflag) {
        //     ADC_wait_stable();
        // }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        g_adc_sample_ready = true;
        // HAL_TIM_Base_Stop(&htim2);
        // HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        adc_sampling=0;
        adc_ready=1;
    }
}
