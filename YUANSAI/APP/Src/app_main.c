#include "app_main.h"
#include "lcd_driver.h"

int app_main(void)
{
    LCD_Init();
    LCD_ShowDefault();
    

    while (1) {
        HAL_GPIO_TogglePin(LCD_BL_GPIO_Port, LCD_BL_Pin);
        HAL_Delay(1000);
    }
    return 0;
}
