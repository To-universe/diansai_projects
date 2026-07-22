#include <stdio.h>
#include "app_main.h"
#include "lcd_driver.h"
#include "receive_pic.h"
#include "pic_decode.h"
#include "entropy_calc.h"
#include "eye_detect.h"
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
    EyeDetect_Init();
    float real_e;

    while (1) {
        receive_pic_poll();

        if (receive_pic_is_frame_ready()) {
            jpg_data = receive_pic_get_frame_data();
            jpg_len  = receive_pic_get_frame_len();
            
            if (decode_data(jpg_data, jpg_len, gray_pic) == JDR_OK) {
                
                float abs_e, rel_e;
                entropy_calc_all(gray_pic, &abs_e, &rel_e);
                float edge_e = edge_energy_calc(gray_pic);

                LCD_SetBackColor(WHITE); LCD_SetTextColor(BLACK);

                EyeDetect_Update(abs_e);//计算熵值
                real_e = 0.8*real_e + 0.2*abs_e;
                LCD_UpdateEntropy(real_e, rel_e);        //显示熵值
                LCD_PrepareFrame(gray_pic);             //准备帧缓冲
                LCD_FlushFrame();                       //刷新显示
                LCD_FPSTick();
                LCD_SetBackColor(WHITE);
                LCD_SetTextColor(BLACK);
                LCD_ShowString(8, 300, (EyeDetect_GetState()==EYE_CLOSED) ? "CLOSED" : "OPEN  ");                          //计算帧率并显示
                

                

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
