#include "lcd_cmd.h"
#include "usart.h"
#include <string.h>

#define BTN_BUF_SIZE 8
static volatile uint8_t btn_buf[BTN_BUF_SIZE];
static volatile uint8_t btn_wr = 0;
static volatile uint8_t btn_rd = 0;
static volatile uint8_t btn_cnt = 0;
static volatile uint8_t rx_byte = 0;

void u3_send(uint8_t *d, uint16_t n) {
    HAL_UART_Transmit(&huart3, d, n, 100);
}

void LCD_Init(void) {
    HAL_Delay(500);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART3) return;
    uint8_t b = rx_byte;
    if (b == 0xA1 || b == 0xA2 || b == 0xA3) {
        if (btn_cnt < BTN_BUF_SIZE) {
            btn_buf[btn_wr] = b;
            btn_wr = (btn_wr + 1) % BTN_BUF_SIZE;
            btn_cnt++;
        }
    }
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
}

uint8_t LCD_PollBtn(void) {
    if (btn_cnt == 0) return 0xFF;
    uint8_t b = btn_buf[btn_rd];
    btn_rd = (btn_rd + 1) % BTN_BUF_SIZE;
    btn_cnt--;
    return b;
}

void LCD_SetTextEx(uint8_t scr, uint8_t ctrl, const char *s) {
    uint8_t buf[256];
    uint16_t len = strlen(s);
    int p = 0;
    buf[p++] = 0xEE; buf[p++] = 0xB1; buf[p++] = 0x10;
    buf[p++] = 0x00; buf[p++] = scr;
    buf[p++] = 0x00; buf[p++] = ctrl;
    memcpy(buf + p, s, len); p += len;
    buf[p++] = 0xFF; buf[p++] = 0xFC; buf[p++] = 0xFF; buf[p++] = 0xFF;
    u3_send(buf, p);
}

void LCD_Curve_Clear(uint8_t scr, uint8_t ctrl, uint8_t ch) {
    uint8_t c[] = {0xEE,0xB1,0x33, 0x00,scr, 0x00,ctrl, ch, 0xFF,0xFC,0xFF,0xFF};
    u3_send(c, sizeof(c));
}

void LCD_Curve_AddData(uint8_t scr, uint8_t ctrl, uint8_t ch,
                        uint8_t *d, uint16_t n) {
    uint8_t buf[2048];
    int p = 0;
    buf[p++] = 0xEE; buf[p++] = 0xB1; buf[p++] = 0x32;
    buf[p++] = 0x00; buf[p++] = scr;
    buf[p++] = 0x00; buf[p++] = ctrl;
    buf[p++] = ch;
    buf[p++] = (uint8_t)(n >> 8);
    buf[p++] = (uint8_t)(n & 0xFF);
    memcpy(buf + p, d, n); p += n;
    buf[p++] = 0xFF; buf[p++] = 0xFC; buf[p++] = 0xFF; buf[p++] = 0xFF;
    u3_send(buf, p);
}
