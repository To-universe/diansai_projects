#ifndef __EYE_DETECT_H__
#define __EYE_DETECT_H__

#include <stdint.h>

typedef enum { EYE_OPEN = 0, EYE_CLOSED = 1 } EyeState;

void     EyeDetect_Init(void);
void     EyeDetect_Update(float entropy, float edge_energy);
EyeState EyeDetect_GetState(void);

#endif
