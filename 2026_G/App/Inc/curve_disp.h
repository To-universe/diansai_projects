#ifndef __CURVE_DISP_H__
#define __CURVE_DISP_H__

#include <stdint.h>

void Curve_DrawSpectrumBars(uint8_t screen,
                            uint32_t freq[3], float amp[3], uint8_t count);

#endif