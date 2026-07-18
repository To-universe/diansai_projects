#ifndef __AD9851_H__
#define __AD9851_H__

#include "ADC_FFT_s.h"
#include "app_main.h"
#include <stdint.h>

#define AD9851_SYSCLK 180000000U

void AD9851_Write_Byte(uint8_t word);
void AD9851_RESET(void);
void AD9851_FQ_Pulse(void);
void AD9851_set_Frequency(uint32_t frequency);
void AD9851_Sweepstart(uint32_t* freq_table,uint16_t length);
void AD9851_SweepCallback(void);
void AD9851_SweepStop(void);
void AD9851_SweepFreq_calc(uint32_t f_start,uint32_t f_end);


extern uint32_t SweepFreq_value[AD9851_SWEEP_FREQ_COUNT];
typedef struct{
    uint32_t* freq_table;
    uint16_t length;
    uint16_t index;
    uint8_t isrunningflag;
}AD9851_SWEEP_t;


extern AD9851_SWEEP_t ad9851_sweep;

#endif
