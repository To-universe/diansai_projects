// STM32CubeMX Configuration: DAC should enable Double Data & set to 8-bit format.
// Usage: call `sine_generator_fill_buffer` in DAC's HT & TC callbacks.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SG_FREQ_Q 20
#define SG_STEP_Q 31

typedef struct {
    // public
    float fs;
    bool chirp_running;

    // private
    uint32_t _freq_ctw;
    uint32_t _freq_acc;

    uint32_t _chirp_end_ctw;
    uint32_t _chirp_step_mul;
    uint32_t _chirp_samples_remain;
    uint32_t _chirp_total_samples;
} SineGenerator;

void sine_generator_init(SineGenerator *sg, float fs);
void sine_generator_set_freq(SineGenerator *sg, uint32_t freq);
void sine_generator_chirp_begin(SineGenerator *sg, float begin_freq, float end_freq, uint32_t n);
void sine_generator_single_freq_begin(SineGenerator *sg, float freq);
void sine_generator_fill_buffer(SineGenerator *sg, uint16_t *buffer, uint32_t buffer_size);

