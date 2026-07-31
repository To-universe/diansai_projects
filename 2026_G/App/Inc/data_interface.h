#ifndef __DATA_INTERFACE_H__
#define __DATA_INTERFACE_H__

#include <stdint.h>

#define WAVE_POINTS  960
#define TEXT_LEN      32

typedef struct {
    uint8_t  wave_1c[WAVE_POINTS];
    uint8_t  wave_3c[WAVE_POINTS];
    char     upp[TEXT_LEN];
    char     urms[TEXT_LEN];
    char     freq[TEXT_LEN];
    uint8_t  wave_updated;

    uint32_t sp_freq_val[3];
    float    sp_amp_val[3];
    uint8_t  sp_count;
    char     sp_freq[3][TEXT_LEN];
    char     sp_amp[3][TEXT_LEN];
    uint8_t  spec_updated;

    uint8_t  flag;
    uint8_t  measure;

} TFT_Data_t;

extern TFT_Data_t g_tft_data;

#endif