#include "app_main.h"
#include "lcd_driver.h"

int app_main(void)
{
    LCD_Init();
    LCD_ShowDefault();   /* draws UI */

    while (1) { }
    return 0;
}
