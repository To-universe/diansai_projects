#include "app_main.h"
#include "AD9851.h"

void app_main(void){

    AD9851_RESET();
    AD9851_set_Frequency(30000000);
    while(1){

    }
}
