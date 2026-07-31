#include "data_feed.h"
#include "data_interface.h"
#include <string.h>
#include <stdio.h>

void Data_Feed_Waveform(uint8_t *wave1, uint8_t *wave3,
                        float upp, float urms, uint32_t freq)
{
    memcpy(g_tft_data.wave_1c, wave1, 960);
    memcpy(g_tft_data.wave_3c, wave3, 960);
    sprintf(g_tft_data.upp,  "Upp=%.0fmV",  upp);
    sprintf(g_tft_data.urms, "Urms=%.0fmV", urms);
    sprintf(g_tft_data.freq, "f=%.1fkHz",   freq / 1000.0f);
    g_tft_data.wave_updated = 1;
}

void Data_Feed_Spectrum(float   base_amp,
                        uint8_t h1_order, float h1_amp,
                        uint8_t h2_order, float h2_amp,
                        uint8_t count,    float base_freq)
{
    /* 基波 */
    g_tft_data.sp_freq_val[0] = (uint32_t)base_freq;
    g_tft_data.sp_amp_val[0]  = base_amp;
    sprintf(g_tft_data.sp_freq[0], "%.1fkHz", base_freq / 1000.0f);
    sprintf(g_tft_data.sp_amp[0],  "%.0fmV",  base_amp);

    /* 谐波1 */
    if (count >= 2) {
        uint32_t f1 = (uint32_t)(base_freq * h1_order);
        g_tft_data.sp_freq_val[1] = f1;
        g_tft_data.sp_amp_val[1]  = h1_amp;
        sprintf(g_tft_data.sp_freq[1], "%.1fkHz", f1 / 1000.0f);
        sprintf(g_tft_data.sp_amp[1],  "%.0fmV",  h1_amp);
    } else {
        g_tft_data.sp_freq_val[1] = 0;
        g_tft_data.sp_amp_val[1]  = 0.0f;
    }

    /* 谐波2 */
    if (count >= 3) {
        uint32_t f2 = (uint32_t)(base_freq * h2_order);
        g_tft_data.sp_freq_val[2] = f2;
        g_tft_data.sp_amp_val[2]  = h2_amp;
        sprintf(g_tft_data.sp_freq[2], "%.1fkHz", f2 / 1000.0f);
        sprintf(g_tft_data.sp_amp[2],  "%.0fmV",  h2_amp);
    } else {
        g_tft_data.sp_freq_val[2] = 0;
        g_tft_data.sp_amp_val[2]  = 0.0f;
        g_tft_data.sp_freq[2][0]  = 0;
        g_tft_data.sp_amp[2][0]   = 0;
    }

    g_tft_data.sp_count  = count;
    g_tft_data.spec_updated = 1;
}
