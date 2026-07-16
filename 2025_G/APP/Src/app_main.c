#include "app_main.h"
#include "AD9851.h"
#include "ADC.h"
#include "dsp/transform_functions.h"
#include "stm32_hal_legacy.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"

void app_main(void){

    AD9851_RESET();
    // AD9851_set_Frequency(100);
    AD9851_Sweepstart(SweepFreq_value, AD9851_SWEEP_FREQ_COUNT);
    arm_rfft_init_8192_q15(&fft_instance, 0U, 1U);
    
    while(1){
        if(adc_wait_flag){
            now_tick = HAL_GetTick();
            if(now_tick-stable_tick >=1){
                ADC_sample_start();
                adc_wait_flag=0;
            }
        }

        if(adc_ready){
            fft_calc();
            response_calc(ad9851_sweep.index, ad9851_sweep.freq_table[ad9851_sweep.index]);
            adc_ready=0;
        }

    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM3){
        AD9851_SweepCallback();
        ADC_wait_stable();
    }
}
