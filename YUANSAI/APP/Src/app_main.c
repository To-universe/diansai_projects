#include "app_main.h"
#include "receive_pic.h"
#include "tjpgd.h"

int app_main(void){
    const uint8_t *jpg_data = 0;
    uint16_t jpg_len = 0;

    receive_pic_init();

    while (1) {
        receive_pic_poll();

        if (receive_pic_is_frame_ready() != 0U) {
            jpg_data = receive_pic_get_frame_data();
            jpg_len = receive_pic_get_frame_len();

            (void)jpg_data;
            (void)jpg_len;

            receive_pic_release_frame();
        }
    }
}
