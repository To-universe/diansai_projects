#include "pic_decode.h"
#include "tjpgd.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include "receive_pic.h"

static size_t in_func (JDEC *jdec,uint8_t* buff,size_t nbyte){
    JpegDecodeDevice *dev=(JpegDecodeDevice*)jdec->device;
    uint16_t remain = dev->jpg_len - dev->jpg_pos;
    if(nbyte>remain){
        nbyte = remain;
    }
    if(buff!=0){
        memcpy(buff, dev->jpg_data+dev->jpg_pos,nbyte);
    }
    dev->jpg_pos += nbyte;
    return nbyte;
}

static int out_func(JDEC *jd, void *bitmap, JRECT *rect)
{
    JpegDecodeDevice *dev = (JpegDecodeDevice *)jd->device;
    uint8_t *gray = (uint8_t *)bitmap;

    uint16_t w = rect->right - rect->left + 1;
    uint16_t h = rect->bottom - rect->top + 1;

    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint16_t sx = rect->left + x;
            uint16_t sy = rect->top + y;
            dev->gray[sy * dev->width + sx] = gray[y * w + x];
        }
    }

    return 1;
}

JRESULT decode_data(const uint8_t *jpg_data,uint16_t jpg_len,uint8_t *gray){
    static uint8_t work[3100]; 
    JDEC jdec;
    JpegDecodeDevice dev;
    JRESULT res;

    dev.jpg_data = jpg_data;
    dev.jpg_len = jpg_len;
    dev.jpg_pos = 0;
    dev.gray = gray;
    dev.width = PIC_LEN;
    dev.height = PIC_WID;

    res = jd_prepare(&jdec, in_func, work,3100,&dev);
    if(res!=JDR_OK){
        return res;
    }
    return jd_decomp(&jdec, out_func, 0);
}