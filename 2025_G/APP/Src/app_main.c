#include "app_main.h"
#include "AD9851.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"

void app_main(void){

    AD9851_RESET();
    AD9851_set_Frequency(400000);
    AD9851_Sweepstart(SweepFreq_value, AD9851_SWEEP_FREQ_COUNT);
    while(1){

    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim3){
        AD9851_SweepCallback();
    }
}
