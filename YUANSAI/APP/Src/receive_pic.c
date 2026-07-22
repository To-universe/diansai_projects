#include "receive_pic.h"
#include "usart.h"

typedef enum {
    JPG_WAIT_START = 0,
    JPG_RECEIVING
} JpgRxState;

static uint8_t rx_dma_buf[JPG_RX_DMA_LEN];
static uint8_t jpg_buf[JPG_MAX_LEN];

static volatile uint8_t frame_ready;
static uint16_t rx_read_pos;
static uint16_t jpg_len;
static uint16_t ready_len;
static uint8_t last_byte;
static JpgRxState rx_state;

static void receive_pic_parser_reset(void)
{
    jpg_len = 0;
    rx_state = JPG_WAIT_START;
}

static void receive_pic_parse_byte(uint8_t byte)
{
    if (frame_ready != 0U) {
        last_byte = byte;
        return;
    }

    if (rx_state == JPG_WAIT_START) {
        if ((last_byte == JPG_MARK) && (byte == JPG_START)) {
            jpg_buf[0] = JPG_MARK;
            jpg_buf[1] = JPG_START;
            jpg_len = 2U;
            rx_state = JPG_RECEIVING;
        }

        last_byte = byte;
        return;
    }

    if (jpg_len >= JPG_MAX_LEN) {
        receive_pic_parser_reset();
        last_byte = byte;
        return;
    }

    jpg_buf[jpg_len] = byte;
    jpg_len++;

    if ((last_byte == JPG_MARK) && (byte == JPG_END)) {
        ready_len = jpg_len;
        frame_ready = 1U;
        receive_pic_parser_reset();
    }

    last_byte = byte;
}

ReceivePicStatus receive_pic_init(void)
{
    rx_read_pos = 0;
    frame_ready = 0;
    ready_len = 0;
    last_byte = 0;
    receive_pic_parser_reset();

    if (HAL_UART_Receive_DMA(&huart2, rx_dma_buf, JPG_RX_DMA_LEN) != HAL_OK) {
        return RECEIVE_PIC_ERROR;
    }

    if (huart2.hdmarx != 0) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    return RECEIVE_PIC_OK;
}

void receive_pic_poll(void)
{
    uint16_t rx_write_pos;

    if (huart2.hdmarx == 0) {
        return;
    }

    rx_write_pos = (uint16_t)(JPG_RX_DMA_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
    if (rx_write_pos >= JPG_RX_DMA_LEN) {
        rx_write_pos = 0;
    }

    while (rx_read_pos != rx_write_pos) {
        receive_pic_parse_byte(rx_dma_buf[rx_read_pos]);

        rx_read_pos++;
        if (rx_read_pos >= JPG_RX_DMA_LEN) {
            rx_read_pos = 0;
        }
    }
}

uint8_t receive_pic_is_frame_ready(void)
{
    return frame_ready;
}

const uint8_t *receive_pic_get_frame_data(void)
{
    if (frame_ready == 0U) {
        return 0;
    }

    return jpg_buf;
}

uint16_t receive_pic_get_frame_len(void)
{
    if (frame_ready == 0U) {
        return 0;
    }

    return ready_len;
}

void receive_pic_release_frame(void)
{
    frame_ready = 0;
    ready_len = 0;
}
