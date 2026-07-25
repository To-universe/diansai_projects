#ifndef __HANNING_WINDOW_Q15_H__
#define __HANNING_WINDOW_Q15_H__


#include "app_main.h"
#include "arm_math.h"

#define HANNING_WINDOW_Q15_SIZE_4096 (4096U)
#define HANNING_WINDOW_Q15_SUM_4096 (67092436LL)
#define HANNING_WINDOW_Q15_SIZE_1024 (1024U)
#define HANNING_WINDOW_Q15_SUM_1024 (16760832LL)

extern const q15_t hanning_window_q15_4096[HANNING_WINDOW_Q15_SIZE_4096];
extern const q15_t hanning_window_q15_1024[HANNING_WINDOW_Q15_SIZE_1024];

#endif
