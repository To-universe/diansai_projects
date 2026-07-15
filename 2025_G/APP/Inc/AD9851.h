#ifndef __AD9851_H__
#define __AD9851_H__

#include "app_main.h"
#include <stdint.h>

#define AD9851_SYSCLK 180000000U
#define AD9851_SWEEP_FREQ_COUNT 416U

void AD9851_Write_Byte(uint8_t word);
void AD9851_RESET(void);
void AD9851_FQ_Pulse(void);
void AD9851_set_Frequency(uint32_t frequency);
void AD9851_SweepFreq(uint32_t f_start,uint32_t f_end);


extern const uint32_t SweepFreq_value[AD9851_SWEEP_FREQ_COUNT];

#endif
