#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include <stdint.h>
#include "main.h"

/* ======================== Screen parameters ======================== */
#define LCD_WIDTH   240
#define LCD_HEIGHT  320
#define SRC_W       48
#define SRC_H       36
#define IMG_W       (SRC_W * 3)
#define IMG_H       (SRC_H * 3)
#define IMG_X       48
#define IMG_Y       10

/* Backward compat: old names used by bsp_xpt2046_lcd.c */
#define LCD_X_LENGTH   LCD_HEIGHT
#define LCD_Y_LENGTH   LCD_WIDTH

/* ======================== FSMC address map ======================== */
#define FSMC_BANK1_BASE     0x60000000UL
#define FSMC_RS_OFFSET      0x00020000UL

#define LCD_CMD              (*(__IO uint16_t *)(FSMC_BANK1_BASE))
#define LCD_DATA             (*(__IO uint16_t *)(FSMC_BANK1_BASE | FSMC_RS_OFFSET))

/* ======================== RGB565 colors ======================== */
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

extern sFONT Font8x16;

/* ======================== Core functions ======================== */
void     LCD_WriteCmd(uint8_t cmd);
void     LCD_WriteData(uint16_t data);
void     LCD_WriteReg(uint8_t cmd, uint16_t data);
uint16_t LCD_ReadData(void);
uint16_t LCD_ReadReg(uint8_t cmd);

void     LCD_Init(void);
void     LCD_Reset(void);

void     LCD_SetWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

void     LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void     LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void     LCD_Clear(uint16_t color);

void     LCD_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pixels);

void     LCD_SetTextColor(uint16_t color);
void     LCD_SetBackColor(uint16_t color);
void     LCD_ShowChar(uint16_t x, uint16_t y, char ch);
void     LCD_ShowString(uint16_t x, uint16_t y, const char *str);

void     LCD_BackLight_On(void);
void     LCD_BackLight_Off(void);

/* ======================== Frame buffer ======================== */
void     LCD_PrepareFrame(const uint8_t *gray_data);
void     LCD_FlushFrame(void);

/* ======================== UI ======================== */
void     LCD_ShowDefault(void);

/* ======================== Backward compat wrappers ======================== */

extern uint8_t LCD_SCAN_MODE;

static inline void LCD_SetFont(sFONT *fonts) { (void)fonts; }
static inline sFONT *LCD_GetFont(void) { return &Font8x16; }
static inline void LCD_SetColors(uint16_t TextColor, uint16_t BackColor) {
    LCD_SetTextColor(TextColor);
    LCD_SetBackColor(BackColor);
}
static inline void ILI9341_GramScan(uint8_t ucOption) { LCD_SCAN_MODE = ucOption; }
static inline void ILI9341_Clear(uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight) {
    LCD_FillRect(usX, usY, usWidth, usHeight, BLACK);
}
static inline void ILI9341_DispStringLine_EN(uint16_t line, char *pStr) {
    LCD_ShowString(0, line, pStr);
}

#endif /* __LCD_DRIVER_H */