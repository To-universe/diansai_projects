#include "data_feed.h"
#include "data_interface.h"
#include <string.h>
#include <stdio.h>

/* Helper: format kHz with 1 decimal without %f (newlib-nano drops %f support) */
static void fmt_khz(char *buf, const char *prefix, uint32_t freq_hz)
{
    uint32_t khz_x10 = (freq_hz + 50UL) / 100UL;  /* kHz * 10, rounded */
    sprintf(buf, "%s%lu.%lukHz", prefix, khz_x10 / 10UL, khz_x10 % 10UL);
}

void Data_Feed_Waveform(uint8_t *wave1, uint8_t *wave3,
                        float upp, float urms, uint32_t freq)
{
    memcpy(g_tft_data.wave_1c, wave1, 960);
    memcpy(g_tft_data.wave_3c, wave3, 960);

    /* FIX: newlib-nano drops %%f, use %%d with manual rounding */
    sprintf(g_tft_data.upp,  "Upp=%dmV",  (int)(upp + 0.5f));
    sprintf(g_tft_data.urms, "Urms=%dmV", (int)(urms + 0.5f));
    fmt_khz(g_tft_data.freq, "f=", freq);

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
    fmt_khz(g_tft_data.sp_freq[0], "", (uint32_t)base_freq);
    sprintf(g_tft_data.sp_amp[0],  "%dmV",  (int)(base_amp + 0.5f));

    /* 谐波1 */
    if (count >= 2) {
        uint32_t f1 = (uint32_t)(base_freq * h1_order);
        g_tft_data.sp_freq_val[1] = f1;
        g_tft_data.sp_amp_val[1]  = h1_amp;
        fmt_khz(g_tft_data.sp_freq[1], "", f1);
        sprintf(g_tft_data.sp_amp[1],  "%dmV",  (int)(h1_amp + 0.5f));
    } else {
        g_tft_data.sp_freq_val[1] = 0;
        g_tft_data.sp_amp_val[1]  = 0.0f;
    }

    /* 谐波2 */
    if (count >= 3) {
        uint32_t f2 = (uint32_t)(base_freq * h2_order);
        g_tft_data.sp_freq_val[2] = f2;
        g_tft_data.sp_amp_val[2]  = h2_amp;
        fmt_khz(g_tft_data.sp_freq[2], "", f2);
        sprintf(g_tft_data.sp_amp[2],  "%dmV",  (int)(h2_amp + 0.5f));
    } else {
        g_tft_data.sp_freq_val[2] = 0;
        g_tft_data.sp_amp_val[2]  = 0.0f;
        g_tft_data.sp_freq[2][0]  = 0;
        g_tft_data.sp_amp[2][0]   = 0;
    }

    g_tft_data.sp_count  = count;
    g_tft_data.spec_updated = 1;
}