#ifndef __MODE_CTRL_H__
#define __MODE_CTRL_H__

#include <stdint.h>

void Mode_Init(void);
void Mode_OnButton(uint8_t btn_data);
void Mode_Update(void);

#endif
