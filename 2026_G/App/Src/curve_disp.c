#include "curve_disp.h"
#include "lcd_cmd.h"
#include "usart.h"
#include <string.h>

/* store previous bars for erase */
static uint16_t prev_x[3], prev_y[3], prev_h[3];
static uint8_t  prev_cnt;

static void SetColor(uint16_t c) {
    uint8_t buf[] = {0xEE,0x41, (uint8_t)(c>>8), (uint8_t)(c&0xFF), 0xFF,0xFC,0xFF,0xFF};
    HAL_UART_Transmit(&huart3, buf, sizeof(buf), 100);
}

static void FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t x2 = x + w - 1;
    uint16_t y2 = y + h - 1;
    uint8_t buf[] = {
        0xEE, 0x55,
        (uint8_t)(x >> 8),  (uint8_t)(x & 0xFF),
        (uint8_t)(y >> 8),  (uint8_t)(y & 0xFF),
        (uint8_t)(x2 >> 8), (uint8_t)(x2 & 0xFF),
        (uint8_t)(y2 >> 8), (uint8_t)(y2 & 0xFF),
        0xFF, 0xFC, 0xFF, 0xFF
    };
    HAL_UART_Transmit(&huart3, buf, sizeof(buf), 100);
}

void Curve_DrawSpectrumBars(uint8_t screen,
                            uint32_t freq[3], float amp[3], uint8_t count)
{
    (void)screen;

    if (count < 1 || count > 3) { prev_cnt = 0; return; }

    const uint16_t BASE_Y  = 540;
    const uint16_t MAX_H   = 320;
    const uint16_t BAR_W   = 20;
    const uint16_t X_START = 60;
    const uint16_t X_RANGE = 500;

    float max_amp = amp[0];
    uint32_t max_freq = freq[0];
    for (uint8_t i = 1; i < count; i++) {
        if (amp[i] > max_amp)   max_amp = amp[i];
        if (freq[i] > max_freq) max_freq = freq[i];
    }
    if (max_amp < 0.001f || max_freq == 0) { prev_cnt = 0; return; }

    uint16_t new_x[3], new_y[3], new_h[3];
    uint8_t changed = (prev_cnt != count);
    for (uint8_t i = 0; i < count; i++) {
        uint16_t x = X_START + (uint16_t)((float)freq[i] / (float)max_freq * X_RANGE);
        uint16_t h = (uint16_t)(amp[i] / max_amp * MAX_H);
        if (h < 1) h = 1;
        uint16_t y = BASE_Y - h;

        new_x[i] = x; new_y[i] = y; new_h[i] = h;
        if (!changed && (x != prev_x[i] || y != prev_y[i] || h != prev_h[i])) {
            changed = 1U;
        }
    }

    if (changed && prev_cnt > 0) {
        SetColor(0xFFFF);  /* white = background */
        for (uint8_t i = 0; i < prev_cnt; i++)
            FillRect(prev_x[i], prev_y[i], 20, prev_h[i]);
    }

    /* draw current bars every time; raw drawings can be cleared by page refresh */
    SetColor(0x07FF);
    for (uint8_t i = 0; i < count; i++) {
        prev_x[i] = new_x[i]; prev_y[i] = new_y[i]; prev_h[i] = new_h[i];
        FillRect(new_x[i], new_y[i], BAR_W, new_h[i]);
    }
    prev_cnt = count;
}
