#include "app_main.h"
#include "lcd_driver.h"
#include "receive_pic.h"
#include "pic_decode.h"

int app_main(void)
{
    uint8_t gray_pic[PIC_SIZE];
    const uint8_t *jpg_data;
    uint16_t jpg_len;

    receive_pic_init();
    LCD_Init();
    LCD_ShowDefault();

    while (1) {
        receive_pic_poll();

        if (receive_pic_is_frame_ready()) {
            jpg_data = receive_pic_get_frame_data();
            jpg_len  = receive_pic_get_frame_len();

            if (decode_data(jpg_data, jpg_len, gray_pic) == JDR_OK) {
                LCD_PrepareFrame(gray_pic);
                LCD_FlushFrame();
            }
            
            receive_pic_release_frame();
        }

        HAL_GPIO_TogglePin(LCD_BL_GPIO_Port, LCD_BL_Pin);
        HAL_Delay(1);
    }
    return 0;
}
