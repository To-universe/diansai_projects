#ifndef __PIC_DECODE_H__
#define __PIC_DECODE_H__

#include "app_main.h"
#include "tjpgd.h"

typedef struct {
    const uint8_t *jpg_data;
    uint16_t jpg_len;
    uint16_t jpg_pos;
    uint8_t *gray;
    uint16_t width;
    uint16_t height;
} JpegDecodeDevice;

JRESULT decode_data(const uint8_t *jpg_data,uint16_t jpg_len,uint8_t *gray);
#endif