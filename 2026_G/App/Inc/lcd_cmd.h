#ifndef __LCD_CMD_H__
#define __LCD_CMD_H__

#include <stdint.h>

void LCD_Init(void);
void LCD_SetTextEx(uint8_t screen_id, uint8_t ctrl_id, const char *text);
void LCD_Curve_Clear(uint8_t screen_id, uint8_t ctrl_id, uint8_t channel);
void LCD_Curve_AddData(uint8_t screen_id, uint8_t ctrl_id, uint8_t channel,
                       uint8_t *data, uint16_t len);
uint8_t LCD_PollBtn(void);
void LCD_ClearText(uint8_t scr, uint8_t ctrl);
#endif
