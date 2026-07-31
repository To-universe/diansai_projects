#ifndef __DATA_FEED_H__
#define __DATA_FEED_H__

#include <stdint.h>

void Data_Feed_Waveform(uint8_t *wave1, uint8_t *wave3,
                        float upp, float urms, uint32_t freq);

void Data_Feed_Spectrum(float   base_amp,
                        uint8_t h1_order, float h1_amp,
                        uint8_t h2_order, float h2_amp,
                        uint8_t count,    float base_freq);

#endif
