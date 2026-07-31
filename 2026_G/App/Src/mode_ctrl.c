#include "main.h"
#include "mode_ctrl.h"
#include "tft_config.h"
#include "lcd_cmd.h"
#include "curve_disp.h"
#include "data_interface.h"

volatile SystemMode_t g_mode = MODE_IDLE;

void Mode_Init(void) {
    g_mode = MODE_IDLE;
}

void Mode_OnButton(uint8_t data) {
    if (data == BTN_DATA_WAVE)  g_mode = MODE_WAVEFORM;
    if (data == BTN_DATA_SPEC)  g_mode = MODE_SPECTRUM;
    if (data == 0xA3)           g_mode = MODE_IDLE;
    if (data == 0xA4)           g_tft_data.flag = 1;  /*校准开启*/
}

static void Mode_Waveform_Run(void) {
    if (!g_tft_data.wave_updated) return;
    g_tft_data.wave_updated = 0;

    LCD_Curve_Clear(SCR_WAVE_1CYCLE, CTRL_GRAPH, CHANNEL_MAIN);
    LCD_Curve_AddData(SCR_WAVE_1CYCLE, CTRL_GRAPH, CHANNEL_MAIN,
                      g_tft_data.wave_1c, WAVE_POINTS);

    LCD_Curve_Clear(SCR_WAVE_3CYCLE, CTRL_GRAPH, CHANNEL_MAIN);
    LCD_Curve_AddData(SCR_WAVE_3CYCLE, CTRL_GRAPH, CHANNEL_MAIN,
                      g_tft_data.wave_3c, WAVE_POINTS);

    LCD_SetTextEx(SCR_WAVE_1CYCLE, CTRL_FREQ, g_tft_data.freq);
    LCD_SetTextEx(SCR_WAVE_1CYCLE, CTRL_UPP,  g_tft_data.upp);
    LCD_SetTextEx(SCR_WAVE_1CYCLE, CTRL_URMS, g_tft_data.urms);
    LCD_SetTextEx(SCR_WAVE_3CYCLE, CTRL_FREQ, g_tft_data.freq);
    LCD_SetTextEx(SCR_WAVE_3CYCLE, CTRL_UPP,  g_tft_data.upp);
    LCD_SetTextEx(SCR_WAVE_3CYCLE, CTRL_URMS, g_tft_data.urms);
}

static void Mode_Spectrum_Run(void) {
    if (!g_tft_data.spec_updated) return;
    g_tft_data.spec_updated = 0;

    Curve_DrawSpectrumBars(SCR_SPECTRUM,
                           g_tft_data.sp_freq_val,
                           g_tft_data.sp_amp_val,
                           g_tft_data.sp_count);

    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_FREQ0, g_tft_data.sp_freq[0]);
    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_AMP0,  g_tft_data.sp_amp[0]);
    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_FREQ1, g_tft_data.sp_freq[1]);
    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_AMP1,  g_tft_data.sp_amp[1]);
    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_FREQ2, g_tft_data.sp_freq[2]);
    LCD_SetTextEx(SCR_SPECTRUM, CTRL_SP_AMP2,  g_tft_data.sp_amp[2]);
}

void Mode_Update(void) {
    static uint32_t last = 0;
    if (last == 0) last = HAL_GetTick();
    if (HAL_GetTick() - last < 1000) return;
    last = HAL_GetTick();

    switch (g_mode) {
        case MODE_IDLE:     break;
        case MODE_WAVEFORM: Mode_Waveform_Run(); break;
        case MODE_SPECTRUM: Mode_Spectrum_Run(); break;
    }
}