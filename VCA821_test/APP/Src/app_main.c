#include "app_main.h"
#include "VCA821.h"
#include "dac.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

void app_main(void){
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    set_VG(680);
    // uint16_t voltage = 100;
    while (1) {
        // set_VG(voltage);
        // voltage+=100;
        // if(voltage>=1000){voltage=100;}
        // HAL_Delay(2000);
    }
}
