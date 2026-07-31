#ifndef __FPGA_H__
#define __FPGA_H__

#include <stdint.h>

#define FPGA_SAMPLE_COUNT 4096U
#define Fs_FPGA 2047812.5f

void fpga_reset(void);
void fpga_start_capture(void);
void fpga_signal_data_ready(void);
uint8_t fpga_is_data_ready(void);
void fpga_clear_data_ready(void);
uint8_t fpga_receive(void);
const int16_t *fpga_get_buffer(void);

#endif
