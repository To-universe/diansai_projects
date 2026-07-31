#ifndef __DAC_SINE_LUT_H__
#define __DAC_SINE_LUT_H__

#include <stdint.h>

#define DAC_SINE_10K_SAMPLE_RATE_HZ  12000000U
#define DAC_SINE_10K_FREQ_HZ         10000U
#define DAC_SINE_10K_POINTS          1200U
#define DAC_SINE_10K_VREF_MV         3300U
#define DAC_SINE_10K_VPP_MV          200U
#define DAC_SINE_10K_MID_CODE        2048U
#define DAC_SINE_10K_AMP_CODE        124U

extern uint16_t dac3_sine_10k_200mvpp_lut[DAC_SINE_10K_POINTS];

#define DAC_SINE_10K_4M_SAMPLE_RATE_HZ  4000000U
#define DAC_SINE_10K_400_POINTS         400U

extern uint16_t dac3_sine_10k_200mvpp_400pt_lut[DAC_SINE_10K_400_POINTS];

#endif
