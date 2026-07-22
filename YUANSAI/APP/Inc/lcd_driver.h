#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include <stdint.h>
#include "main.h"

/* ======================== Screen ======================== */
#define LCD_WIDTH   240
#define LCD_HEIGHT  320

/* ======================== FSMC addresses ======================== */
#define FSMC_BANK1_BASE     0x60000000UL
#define FSMC_RS_OFFSET      0x00020000UL

#define LCD_CMD   (*(__IO uint16_t *)(FSMC_BANK1_BASE))
#define LCD_DATA  (*(__IO uint16_t *)(FSMC_BANK1_BASE | FSMC_RS_OFFSET))

/* ======================== RGB565 Colors ======================== */
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define GRAY        0x8410
#define DARKGRAY    0x4208

/* ======================== Font ======================== */
typedef struct {
    const uint8_t *table;
    uint16_t Width;
    uint16_t Height;
} sFONT;


/* ======================== Low-level FSMC ======================== */
void     LCD_WriteCmd(uint8_t cmd);
void     LCD_WriteData(uint16_t data);
void     LCD_WriteReg(uint8_t cmd, uint16_t data);

/* ======================== Init ======================== */
void     LCD_Init(void);
void     LCD_Reset(void);

/* ======================== Drawing ======================== */
void     LCD_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void     LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void     LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void     LCD_Clear(uint16_t color);
void     LCD_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);

/* ======================== Text ======================== */
void     LCD_SetTextColor(uint16_t color);
void     LCD_SetBackColor(uint16_t color);
void     LCD_ShowChar(uint16_t x, uint16_t y, char ch);
void     LCD_ShowString(uint16_t x, uint16_t y, const char *str);

/* ======================== Backlight ======================== */
void     LCD_BackLight_On(void);
void     LCD_BackLight_Off(void);

/* ======================== High-level ======================== */
void     LCD_ShowDefault(void);

#endif