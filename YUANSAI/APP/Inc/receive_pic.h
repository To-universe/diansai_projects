#ifndef __RECEIVE_PIC_H__
#define __RECEIVE_PIC_H__

#include <stdint.h>

#define PIC_LEN 48U
#define PIC_WID 36U
#define PIC_SIZE (PIC_LEN * PIC_WID)

#define JPG_MARK 0xFFU
#define JPG_START 0xD8U
#define JPG_END 0xD9U

#define JPG_MAX_LEN 4096U
#define JPG_RX_DMA_LEN 512U

typedef enum {
    RECEIVE_PIC_OK = 0,
    RECEIVE_PIC_ERROR = -1,
    RECEIVE_PIC_BUSY = -2
} ReceivePicStatus;

ReceivePicStatus receive_pic_init(void);
void receive_pic_poll(void);
uint8_t receive_pic_is_frame_ready(void);
const uint8_t *receive_pic_get_frame_data(void);
uint16_t receive_pic_get_frame_len(void);
void receive_pic_release_frame(void);

#endif
