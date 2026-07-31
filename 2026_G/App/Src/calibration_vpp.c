#include "calibration_vpp.h"
#include "arm_math_types.h"
#include <math.h>
#include <stdint.h>

static vpp_cal_table_t g_vpp_cal;
static uint8_t g_vpp_cal_enabled = 1U;

static const float32_t freq_hz_init[VPP_CAL_MAX_POINTS] = {10000.000000f,
    13000.000000f,
    16500.000000f,
    21500.000000f,
    28000.000000f,
    36000.000000f,
    46500.000000f,
    60000.000000f,
    77500.000000f,
    100000.000000f,
    109000.000000f,
    118500.000000f,
    129000.000000f,
    140500.000000f,
    152500.000000f,
    166000.000000f,
    181000.000000f,
    197000.000000f,
    214500.000000f,
    233500.000000f,
    254000.000000f,
    276500.000000f,
    301000.000000f,
    327500.000000f,
    356500.000000f,
    388000.000000f,
    422000.000000f,
    459500.000000f,
    500000.000000f
};
static const float32_t gain_corr_init[VPP_CAL_MAX_POINTS] = {
    1.000000000f,
    1.000695894f,
    1.000695894f,
    1.000695894f,
    1.001392758f,
    1.002789400f,
    1.004189944f,
    1.004189944f,
    1.007002801f,
    1.010541110f,
    1.011963406f,
    1.015536723f,
    1.019858156f,
    1.024216524f,
    1.027877055f,
    1.034532374f,
    1.042028986f,
    1.048869439f,
    1.056576046f,
    1.063609467f,
    1.071535022f,
    1.078769693f,
    1.089393939f,
    1.096036586f,
    1.106153846f,
    1.114728682f,
    1.121684868f,
    1.132283464f,
    1.142176330f
};

void vpp_cal_table_init(void){
    g_vpp_cal.count = VPP_CAL_MAX_POINTS;
    g_vpp_cal_enabled = 1U;
    for(uint8_t i = 0 ;i<VPP_CAL_MAX_POINTS;i++){
        g_vpp_cal.freq_hz[i]=freq_hz_init[i];
        g_vpp_cal.gain_corr[i]=gain_corr_init[i];
    }
}

void vpp_cal_set_enabled(uint8_t enabled)
{
    g_vpp_cal_enabled = enabled ? 1U : 0U;
}

uint8_t vpp_cal_is_enabled(void)
{
    return g_vpp_cal_enabled;
}

float32_t cal_corr(float32_t f){
    if (!g_vpp_cal_enabled) {
        return 1.0f;
    }

    if (g_vpp_cal.count == 0U || f <= 0.0f) {
        return 1.0f;
    }

    if (g_vpp_cal.count == 1U || f <= g_vpp_cal.freq_hz[0]) {
        return g_vpp_cal.gain_corr[0];
    }

    uint8_t last = (uint8_t)(g_vpp_cal.count - 1U);
    if (f >= g_vpp_cal.freq_hz[last]) {
        return g_vpp_cal.gain_corr[last];
    }

    float32_t x = logf(f);
    for (uint8_t i = 0U; i < last; i++) {
        float32_t f0 = g_vpp_cal.freq_hz[i];
        float32_t f1 = g_vpp_cal.freq_hz[i + 1U];

        if (f >= f0 && f <= f1) {
            float32_t x0 = logf(f0);
            float32_t x1 = logf(f1);
            float32_t y0 = g_vpp_cal.gain_corr[i];
            float32_t y1 = g_vpp_cal.gain_corr[i + 1U];
            float32_t dx = x1 - x0;

            if (fabsf(dx) < 1.0e-12f) {
                return y0;
            }

            float32_t t = (x - x0) / dx;
            return y0 + t * (y1 - y0);
        }
    }

    return g_vpp_cal.gain_corr[last];
}
