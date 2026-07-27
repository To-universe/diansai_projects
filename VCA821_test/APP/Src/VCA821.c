#include "VCA821.h"
#include "dac.h"
#include "stm32g4xx_hal_dac.h"
#include <stdint.h>

#define VREF 3300.0

void set_VG(uint16_t voltage)
{
    // if (voltage < 0.0f) {
    //     voltage = 0.0f;
    // } else if (voltage > 3.3f) {
    //     voltage = 3.3f;
    // }
    float Vol;
    Vol=0.29194*voltage+12.46779;   //自行拟合的参数
    // Vol=0.2809*voltage+11.728;      //例程给的参数
    uint16_t dac_word = (uint16_t)((Vol * 4095.0f / VREF));
    
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_word);
}
