#ifndef __TFT_CONFIG_H__
#define __TFT_CONFIG_H__

/* screen */
#define SCR_HOME         0
#define SCR_WAVE_1CYCLE  1
#define SCR_WAVE_3CYCLE  2
#define SCR_SPECTRUM     3

/* curve */
#define CTRL_GRAPH       1

/* Screen1/2 text */
#define CTRL_UPP        17
#define CTRL_URMS       18
#define CTRL_FREQ       19

/* Screen3 text */
#define CTRL_SP_FREQ0    2
#define CTRL_SP_AMP0     4
#define CTRL_SP_FREQ1    5
#define CTRL_SP_AMP1     6
#define CTRL_SP_FREQ2   14
#define CTRL_SP_AMP2    15

/* button */
#define BTN_START_MEAS  11
#define BTN_BACK         8

/* button data */
#define BTN_DATA_IDLE   0x00
#define BTN_DATA_WAVE   0xA1
#define BTN_DATA_SPEC   0xA2

#define CURVE_DISP_POINTS  960
#define CHANNEL_MAIN         0

typedef enum {
    MODE_IDLE     = 0,
    MODE_WAVEFORM = 1,
    MODE_SPECTRUM = 2
} SystemMode_t;

#endif
