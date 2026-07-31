#include "app_main.h"
#include "arm_math_types.h"
#include "calc.h"
#include "calibration_vpp.h"
#include "data_feed.h"
#include "fft.h"
#include "fpga.h"
#include "uart_printf.h"
#include "voltage.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lcd_cmd.h"
#include "mode_ctrl.h"
#include "main.h"
#include "data_interface.h"

typedef enum {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_MEASURE,
    STATE_DISPLAY,
    STATE_SCREEN_TIME,
    STATE_SCREEN_FREQ,
} app_state_t;

typedef struct {
    app_state_t state;
    bool calibration_mode;
    calc_accumulator_t acc;
    bool display_ready;
    float32_t vrms_mean;
    float32_t vpp_mean;
    float32_t f0;
    float32_t f0_used;
    float32_t harmonic_means[3];
    uint8_t  valid_counts[3];
    uint8_t  harmonic_orders[3];
    uint8_t  harmonic_order_count;
    float32_t waveform[WAVEFORM_SIZE];
    float32_t waveform_3_cycles[WAVEFORM_SIZE];
    uint8_t waveform_u8[WAVEFORM_SIZE];
    uint8_t waveform_3_cycles_u8[WAVEFORM_SIZE];

} app_ctx_t;

/* newlib-nano drops %f support; use this instead */
static void print_f32(float32_t v, uint8_t int_width, uint8_t frac_digits)
{
    if (v < 0.0f) {
        printf("-");
        v = -v;
    }
    int32_t ipart = (int32_t)v;
    float32_t frac = v - (float32_t)ipart;
    uint32_t scale = 1U;
    for (uint8_t d = 0U; d < frac_digits; d++) {
        scale *= 10U;
    }
    uint32_t fpart = (uint32_t)(frac * (float32_t)scale + 0.5f);
    if (fpart >= scale) {
        ipart++;
        fpart -= scale;
    }
    printf("%*ld.%0*lu", int_width, (long)ipart, frac_digits, (unsigned long)fpart);
}

static void process_input(app_ctx_t *ctx)
{
    static char cmd[16];
    static uint8_t cmd_len = 0U;

    uint8_t ch;
    while (HAL_UART_Receive(&huart5, &ch, 1U, 1U) == HAL_OK) {
        printf("%c\n", ch);
        if (ch >= 'a' && ch <= 'z') {
            ch = (uint8_t)(ch - 'a' + 'A');
        }

        if (ch == 'C') {
            ctx->calibration_mode = true;
            vpp_cal_set_enabled(0U);
            cmd_len = 0U;
            printf("[mode] CALIBRATION\r\n");
            continue;
        }

        if (ch == 'R' || ch == 'N') {
            ctx->calibration_mode = false;
            vpp_cal_set_enabled(1U);
            cmd_len = 0U;
            printf("[mode] NORMAL\r\n");
            continue;
        }

        if (ch == 'M' && ctx->state == STATE_IDLE) {
            cmd_len = 0U;
            printf("[cmd] MEAS\r\n");
            ctx->state = STATE_MEASURE;
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            cmd[cmd_len] = '\0';
            if (strcmp(cmd, "CAL") == 0 || strcmp(cmd, "CALIB") == 0 || strcmp(cmd, "CALIBRATE") == 0) {
                ctx->calibration_mode = true;
                vpp_cal_set_enabled(0U);
                printf("[mode] CALIBRATION\r\n");
            } else if (strcmp(cmd, "AUTO") == 0 || strcmp(cmd, "NORMAL") == 0 || strcmp(cmd, "RUN") == 0) {
                ctx->calibration_mode = false;
                vpp_cal_set_enabled(1U);
                printf("[mode] NORMAL\r\n");
            } else if (ctx->state == STATE_IDLE &&
                       (strcmp(cmd, "MEAS") == 0 || strcmp(cmd, "START") == 0 || strcmp(cmd, "M") == 0)) {
                printf("[cmd] MEAS\r\n");
                ctx->state = STATE_MEASURE;
            }
            cmd_len = 0U;
            continue;
        }

        if (cmd_len < sizeof(cmd) - 1U) {
            cmd[cmd_len++] = (char)ch;
        } else {
            cmd_len = 0U;
        }
    }

    if (ctx->state == STATE_IDLE && !ctx->calibration_mode) {
        ctx->state = STATE_MEASURE;
    }
}

static void waveform_to_screen_q7(const float32_t *src, uint8_t *dst, uint16_t len)
{
    if (len == 0U) {
        return;
    }

    float32_t min_v = src[0];
    float32_t max_v = src[0];
    for (uint16_t k = 1U; k < len; k++) {
        if (src[k] < min_v) {
            min_v = src[k];
        }
        if (src[k] > max_v) {
            max_v = src[k];
        }
    }

    float32_t center = 0.5f * (max_v + min_v);
    float32_t half_range = 0.5f * (max_v - min_v);
    if (half_range < 1.0e-9f) {
        memset(dst, 0, len);
        return;
    }

    for (uint16_t k = 0U; k < len; k++) {
        float32_t norm = (src[k] - center) / half_range;
        if (norm > 1.0f) {
            norm = 1.0f;
        } else if (norm < -1.0f) {
            norm = -1.0f;
        }

        float32_t scaled = (norm >= 0.0f) ? (norm * 127.0f) : (norm * 128.0f);
        int32_t q = (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
        if (q > 127) {
            q = 127;
        } else if (q < -128) {
            q = -128;
        }
        dst[k] = (uint8_t)((int8_t)q);
    }
}

static void state_calculation(app_ctx_t *ctx)
{
    // if (ctx->state != STATE_MEASURE) {
    //     return;
    // }

    for (uint8_t i = 0U; i < CALC_FRAME_COUNT; i++) {
        fpga_start_capture();

        while (!fpga_is_data_ready()) {}
        if (!fpga_receive()) {
            Error_Handler();
        }

        // const int16_t *buffer = fpga_get_buffer();
        // for (uint32_t i = 0; i < 4096; i++) {
        //     float value = (float32_t)buffer[i] / (float32_t)(1UL << 15);
        //     print_f32(value, 1, 2);
        //     printf("%c", i % 16 == 15 ? '\n' : ' ');
        // } 

        HAL_Delay(100);

        fft_result_t fft_res;
        fft_compute(fpga_get_buffer(), &fft_res);

        printf("[frame %u] peaks=%u  f0=", i, fft_res.peak_count);
        print_f32(fft_res.fitted_f0, 0, 1);
        printf(" Hz  noise_floor=");
        print_f32(fft_res.noise_floor, 0, 1);
        printf("\r\n");

        vol_result_t vol_res;
        voltage_compute(fpga_get_buffer(), &fft_res, &vol_res);

        calc_accumulate(&ctx->acc, &vol_res);
    }

    float32_t f0_mean;
    float32_t waveform_mean[WAVEFORM_SIZE];

    calc_finalize(&ctx->acc,
                  &ctx->vrms_mean, &ctx->vpp_mean, &f0_mean,
                  ctx->harmonic_means, ctx->valid_counts,
                  waveform_mean);

    ctx->f0 = ctx->acc.f0;
    ctx->f0_used = f0_mean;
    ctx->harmonic_order_count = 0U;
    for (uint16_t k = 0U; k < WAVEFORM_SIZE; k++) {
        ctx->waveform[k] = waveform_mean[k];
        uint16_t idx = (3*k)%WAVEFORM_SIZE;
        ctx->waveform_3_cycles[k] = waveform_mean[idx];
    }
    waveform_to_screen_q7(ctx->waveform, ctx->waveform_u8, WAVEFORM_SIZE);
    waveform_to_screen_q7(ctx->waveform_3_cycles, ctx->waveform_3_cycles_u8, WAVEFORM_SIZE);
    ctx->display_ready = true;

    calc_init(&ctx->acc);
    // ctx->state = STATE_DISPLAY;
}

static void show(app_ctx_t *ctx)
{
    if (!ctx->display_ready) {
        return;
    }

    printf("=== Results ===\r\n\r\n");

    printf("Vrms       : ");
    print_f32(ctx->vrms_mean, 0, 4);
    printf(" V\r\n");

    printf("Vpp        : ");
    print_f32(ctx->vpp_mean, 0, 4);
    printf(" V\r\n");

    printf("Fundamental: ");
    print_f32(ctx->f0, 0, 2);
    printf(" Hz\r\n");

    printf("\r\nOrders     : ");
    if (ctx->harmonic_order_count == 0U) {
        printf("none\r\n");
    } else {
        for (uint8_t i = 0U; i < ctx->harmonic_order_count; i++) {
            printf("H%u", ctx->harmonic_orders[i]);
            if (i + 1U < ctx->harmonic_order_count) {
                printf(", ");
            }
        }
        printf("\r\n");
    }

    printf("\r\nHarmonics:\r\n");
    for (uint8_t i = 1U; i < VOL_AMP_BY_ORDER_SIZE; i++) {
        if (ctx->valid_counts[i] > 0U) {
            printf("  H%-2u : ", i);
            print_f32((float32_t)i * ctx->f0, 8, 2);
            printf(" Hz  ");
            print_f32(ctx->harmonic_means[i], 8, 4);
            printf(" V  (valid %u/%u)\r\n",
                   ctx->valid_counts[i], (unsigned)CALC_FRAME_COUNT);
        }
    }

    printf("\r\nWaveform (256 pts):\r\n  ");
    for (uint16_t k = 0U; k < WAVEFORM_SIZE; k++) {
        print_f32(ctx->waveform[k], 0, 3);
        if (k < WAVEFORM_SIZE - 1U) {
            printf(",");
        }
        if ((k & 7U) == 7U && k < WAVEFORM_SIZE - 1U) {
            printf("\r\n  ");
        }
    }
    printf("\r\n\r\n");

    ctx->display_ready = false;

    /* return to idle so process_input can re-trigger */
    ctx->state = STATE_IDLE;
}

void state_display(app_ctx_t* ctx){
    Data_Feed_Waveform(ctx->waveform_u8, ctx->waveform_3_cycles_u8,(float)ctx->vpp_mean, (float)ctx->vrms_mean, ctx->f0);
    Data_Feed_Spectrum((float)ctx->harmonic_means[0],ctx->harmonic_orders[1], (float32_t)ctx->harmonic_means[1],ctx->harmonic_orders[2],(float)ctx->harmonic_means[2], ctx->harmonic_order_count,(float)ctx->f0);
}

//state transition
#if 0
void update_state(app_ctx_t* ctx){
    switch (ctx->state) {
        case STATE_IDLE:
            if(ctx->calibration_mode){
                //当校准信号发出
            ){
                ctx->state = STATE_CALIBRATING;
            }
            else if(0){
                //当计算信号发出
            ){
                ctx->state = STATE_MEASURE;
            }else{
                ctx->state = STATE_IDLE;
            }
            break;
        case STATE_CALIBRATING:
            ctx->state = STATE_IDLE;
            break;
        case STATE_MEASURE:
            ctx->state =  STATE_DISPLAY;
            break;
        case STATE_DISPLAY:
            ctx->state = STATE_IDLE;
            break;
    }
}
#endif

void update_state(app_ctx_t* ctx){
    switch (ctx->state) {
        case STATE_IDLE:
            if (g_tft_data.flag == 1) {
                ctx->state = STATE_CALIBRATING;
            } else if (g_tft_data.measure == 1) {
                ctx->state = STATE_MEASURE;
            }
            break;
        case STATE_CALIBRATING:
            ctx->state = STATE_IDLE;
            break;
        case STATE_MEASURE:
            ctx->state = STATE_DISPLAY;
            break;
        case STATE_DISPLAY:
            ctx->state = STATE_IDLE;
            break;
        case STATE_SCREEN_TIME:
        case STATE_SCREEN_FREQ:
            break;
    }
}
//state actions
void state_act(app_ctx_t* ctx){
    switch (ctx->state) {
        case STATE_IDLE:
            break;
        case STATE_CALIBRATING:
            //加入校准相关代码

            LCD_SetTextEx(0, 7, "\xD0\xA3\xD1\xE9\xCD\xEA\xB3\xC9");
            g_tft_data.flag = 0;  /*校准完成*/
            break;
        case STATE_MEASURE:
            state_calculation(ctx);
            LCD_SetTextEx(0, 7, "\xB2\xE2\xC1\xBF\xCD\xEA\xB3\xC9");
            g_tft_data.measure = 0;  /*测量完成*/
            break;
        case STATE_DISPLAY:
            state_display(ctx);
            break;
        case STATE_SCREEN_TIME:
        case STATE_SCREEN_FREQ:
            break;
    }
}

void app_main(void)
{
    HAL_Delay(100);
    fpga_reset();
    fft_init();
    vpp_cal_table_init();
    LCD_Init();
    Mode_Init();
    printf("[ready] mode NORMAL; send CAL for calibration mode\r\n");

    app_ctx_t ctx = {0};
    ctx.state = STATE_IDLE;
    ctx.calibration_mode = false;
    calc_init(&ctx.acc);

    while (1) {
        // process_input(&ctx);
        // state_calculation(&ctx);
        // show(&ctx);
        update_state(&ctx);
        state_act(&ctx);

        Mode_OnButton(LCD_PollBtn());
        Mode_Update();
        
        HAL_Delay(50);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == FPGA_IRQN_Pin) {
        fpga_signal_data_ready();
    }
}
