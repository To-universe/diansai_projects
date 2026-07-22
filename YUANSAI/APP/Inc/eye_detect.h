#ifndef __EYE_DETECT_H__
#define __EYE_DETECT_H__

#include <stdint.h>

typedef enum { EYE_OPEN = 0, EYE_CLOSED = 1 } EyeState;

void     EyeDetect_Init(void);
void     EyeDetect_Update(float current_entropy);
EyeState EyeDetect_GetState(void);

#endif
