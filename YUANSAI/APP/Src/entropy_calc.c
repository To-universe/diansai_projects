#include "entropy_calc.h"
#include "receive_pic.h"
#include <math.h>
#include <stdint.h>

uint16_t gray_value_count[256];
float gray_value_p[256];

static void counting_gray(uint8_t* gray_pic){
    for(uint16_t i = 0;i<256;i++){
        gray_value_count[i]=0;
    }
    for(uint16_t i = 0;i<PIC_SIZE;i++){
        uint16_t index = gray_pic[i];
        gray_value_count[index]++;
    }
    for(uint16_t i =0;i<256;i++){
        gray_value_p[i]=(float)gray_value_count[i] / (float)PIC_SIZE;
    }
}

static float entropy_abs_calc(void){
    float res = 0.0;
    for(uint16_t i =0;i<256;i++){
        if(gray_value_p[i]==0  ||  gray_value_p[i]==1){
            continue;
        }

        res -= gray_value_p[i]*log2f(gray_value_p[i]);
    }
    return res;
}

static float entropy_rela_calc(float abs){
    return abs/8.0*100.0;
}

void entropy_calc_all(uint8_t* gray_pic,float* entropy_abs,float* entropy_rel){
    counting_gray(gray_pic);
    float entro_a = entropy_abs_calc();
    float entro_r = entropy_rela_calc(entro_a);

    *entropy_abs = entro_a;
    *entropy_rel = entro_r;
}
