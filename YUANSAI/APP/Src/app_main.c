#include "app_main.h"
#include "receive_pic.h"
#include "stm32f1xx_hal_uart.h"
#include "tjpgd.h"
#include "pic_decode.h"
#include <stdint.h>
#include "lcd_driver.h"
#include "stdio.h"
#include "usart.h"


int app_main(void){
    const uint8_t *jpg_data = 0;
    uint16_t jpg_len = 0;
    uint8_t gray_pic[PIC_SIZE];

    receive_pic_init();


    LCD_Init();
    LCD_ShowDefault();   /* draws UI */

    LCD_FlushFrame();

    while (1) {
        receive_pic_poll();
        if (receive_pic_is_frame_ready()) {
            jpg_data = receive_pic_get_frame_data();
            jpg_len = receive_pic_get_frame_len();

            if(decode_data(jpg_data, jpg_len, gray_pic)==JDR_OK){
                //此处计算熵
                
                //此处调用屏显
                LCD_PrepareFrame(gray_pic);

            }

            receive_pic_release_frame();
        }
        LCD_FlushFrame();
    }
    return 0;
}
