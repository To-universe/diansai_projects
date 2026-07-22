#include <stdio.h>
#include "app_main.h"
#include "lcd_driver.h"
#include "receive_pic.h"
#include "pic_decode.h"
#include "entropy_calc.h"
#include "usart.h"

int app_main(void)
{
    uint8_t gray_pic[PIC_SIZE];
    const uint8_t *jpg_data;
    uint16_t jpg_len;
    uint32_t t0,t1,t2,t3;

    receive_pic_init();
    LCD_Init();
    LCD_ShowDefault();

    while (1) {
        receive_pic_poll();

        if (receive_pic_is_frame_ready()) {
            jpg_data = receive_pic_get_frame_data();
            jpg_len  = receive_pic_get_frame_len();
            t0 = HAL_GetTick();
            if (decode_data(jpg_data, jpg_len, gray_pic) == JDR_OK) {
                t1=HAL_GetTick();
                float abs_e, rel_e;
                entropy_calc_all(gray_pic, &abs_e, &rel_e);//计算熵值
                t2=HAL_GetTick();
                LCD_UpdateEntropy(abs_e, rel_e);        //显示熵值
                LCD_PrepareFrame(gray_pic);             //准备帧缓冲
                LCD_FlushFrame();                       //刷新显示
                LCD_FPSTick();                          //计算帧率并显示
                t3=HAL_GetTick();

                char tbuf[16];
                LCD_SetBackColor(WHITE);
                LCD_SetTextColor(BLACK);
                sprintf(tbuf, "t1-t0:%lums", t1-t0);
                LCD_ShowString(8, 210, tbuf);
                sprintf(tbuf, "t2-t1:%lums", t2-t1);
                LCD_ShowString(8, 230, tbuf);
                sprintf(tbuf, "t3-t2:%lums", t3-t2);
                LCD_ShowString(8, 250, tbuf);
                sprintf(tbuf, "t3-t0:%lums", t3-t0);
                LCD_ShowString(8, 270, tbuf);

                // HAL_UART_Transmit(&huart2, (uint8_t*)&abs_e, 4, 100);
                // HAL_UART_Transmit(&huart2, (uint8_t*)&rel_e, 4, 100);
            }

            receive_pic_release_frame();
        }

        // HAL_GPIO_TogglePin(LCD_BL_GPIO_Port, LCD_BL_Pin);
        HAL_Delay(1);
    }
    return 0;
}
