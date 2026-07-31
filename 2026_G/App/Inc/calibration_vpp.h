#ifndef __CALIBRATION_VPP_H__
#define __CALIBRATION_VPP_H__


#include "arm_math_types.h"
#include <stdint.h>

#define VPP_CAL_MAX_POINTS 29U

typedef struct{
    uint8_t count;
    float32_t freq_hz[VPP_CAL_MAX_POINTS];
    float32_t gain_corr[VPP_CAL_MAX_POINTS];
} vpp_cal_table_t;

void vpp_cal_table_init(void);
void vpp_cal_set_enabled(uint8_t enabled);
uint8_t vpp_cal_is_enabled(void);
float32_t cal_corr(float32_t f);

#endif
