#include "AD9851.h"
#include "ADC_FFT_s.h"
#include "app_main.h"
#include "main.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <sys/_intsup.h>

AD9851_SWEEP_t ad9851_sweep;

uint8_t SF_count=0;
uint32_t SweepFreq_value[AD9851_SWEEP_FREQ_COUNT];

void AD9851_Write_Byte(uint8_t word){
    uint32_t odr = GPIOB->ODR;
    odr = (odr & ~0x000000FFu) | word;
    GPIOB->ODR = odr;

    CLK_GPIO_Port->BSRR = CLK_Pin;
    __NOP();
    CLK_GPIO_Port->BRR = CLK_Pin;
}

void AD9851_RESET(void){
    RST_GPIO_Port->BSRR=RST_Pin;
    __NOP();
    RST_GPIO_Port->BRR=RST_Pin;
}

void AD9851_FQ_Pulse(void){
    FQ_GPIO_Port->BSRR = FQ_Pin;
    __NOP();
    FQ_GPIO_Port->BRR = FQ_Pin;
}

uint32_t AD9851_calc_Frequency(uint32_t frequency){
    return (uint32_t)(((uint64_t)frequency << 32) / AD9851_SYSCLK);
}

void AD9851_set_Frequency(uint32_t frequency){
    uint32_t freq_word = AD9851_calc_Frequency(frequency);
    /* AD9851 parallel load order: W0 control, then W1..W4 frequency MSB to LSB. */
    AD9851_Write_Byte(0x01);
    AD9851_Write_Byte((uint8_t)((freq_word >> 24) & 0xFFu));
    AD9851_Write_Byte((uint8_t)((freq_word >> 16) & 0xFFu));
    AD9851_Write_Byte((uint8_t)((freq_word >> 8) & 0xFFu));
    AD9851_Write_Byte((uint8_t)(freq_word & 0xFFu));
    AD9851_FQ_Pulse();
}

void AD9851_Sweepstart(uint32_t *freq_table, uint16_t length){
    if(freq_table == NULL || length==0){
        return;
    }
    ad9851_sweep.freq_table = freq_table;
    ad9851_sweep.length = length;
    ad9851_sweep.index = 0;
    ad9851_sweep.isrunningflag=1;
    AD9851_set_Frequency(ad9851_sweep.freq_table[ad9851_sweep.index]);
}

void AD9851_SweepCallback(void){
    if(!ad9851_sweep.isrunningflag){
        return;
    }
    
    ad9851_sweep.index++;
    if(ad9851_sweep.index>=ad9851_sweep.length){
        AD9851_SweepStop();
        return;
    }
    AD9851_set_Frequency(ad9851_sweep.freq_table[ad9851_sweep.index]);

}

void AD9851_SweepStop(void){
    ad9851_sweep.isrunningflag = 0;
    HAL_TIM_Base_Stop_IT(&htim3);
}

void AD9851_SweepFreq_calc(uint32_t f_start, uint32_t f_end){
    if(f_start == 0U || f_end == 0U){
        return;
    }

    if(AD9851_SWEEP_FREQ_COUNT == 1U){
        SweepFreq_value[0] = f_start;
        return;
    }

    const float start = (float)f_start;
    const float ratio = (float)f_end / (float)f_start;

    for(uint32_t i = 0; i < AD9851_SWEEP_FREQ_COUNT; i++){
        const float t = (float)i / (float)(AD9851_SWEEP_FREQ_COUNT - 1U);
        const float freq = start * powf(ratio, t);
        SweepFreq_value[i] = (uint32_t)(freq + 0.5f);
    }

    SweepFreq_value[0] = f_start;
    SweepFreq_value[AD9851_SWEEP_FREQ_COUNT - 1U] = f_end;
}

void AD9851_SweepFreq_LineCalc(uint32_t f_start,uint32_t f_end){
    if(f_start == 0U || f_end == 0U){
        return;
    }

    if(AD9851_SWEEP_FREQ_COUNT == 1U){
        SweepFreq_value[0] = f_start;
        return;
    }
    const float start = (float)f_start;
    const float add = ((float)f_end - (float)f_start)/(float)(AD9851_SWEEP_FREQ_COUNT - 1U);
    for(uint32_t i = 0;i<AD9851_SWEEP_FREQ_COUNT;i++){
        const float freq = start + i*add;
        SweepFreq_value[i] = (uint32_t)(freq + 0.5f);
    }
    SweepFreq_value[0] = f_start;
    SweepFreq_value[AD9851_SWEEP_FREQ_COUNT - 1U] = f_end;
}