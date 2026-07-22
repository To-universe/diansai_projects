#ifndef __ENTROPY_CALC_H__
#define __ENTROPY_CALC_H__

#include <stdint.h>
#include "receive_pic.h"


void entropy_calc_all(uint8_t* gray_pic,float* entropy_abs,float* entropy_rel);
extern const float entropy_count_lut[PIC_SIZE + 1U];
#endif
