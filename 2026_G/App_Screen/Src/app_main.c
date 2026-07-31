#include "app_main.h"
#include "lcd_cmd.h"
#include "mode_ctrl.h"
#include "main.h"

void App_Init(void) {
    LCD_Init();
    Mode_Init();
}

void App_Loop(void) {
    Mode_OnButton(LCD_PollBtn());
    Mode_Update();
}