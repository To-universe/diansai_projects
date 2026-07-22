#include "eye_detect.h"
#include <string.h>

#define DELTA_THRESH  0.15f
#define WINDOW        7
#define VOTE_MIN      4

static EyeState output = EYE_OPEN;
static float    last_entropy = 0.0f;
static float    anchor = 0.0f;         /* entropy at last transition */
static uint8_t  vote_buf[WINDOW];
static uint8_t  vote_idx = 0;
static uint8_t  initialized = 0;

void EyeDetect_Init(void)
{
    output = EYE_OPEN;
    last_entropy = 0.0f;
    anchor = 0.0f;
    memset(vote_buf, 0, WINDOW);
    vote_idx = 0;
    initialized = 0;
}

static uint8_t sum_buf(const uint8_t *buf)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < WINDOW; i++) s += buf[i];
    return s;
}

void EyeDetect_Update(float cur)
{
    if (!initialized) {
        last_entropy = cur;
        anchor = cur;
        initialized = 1;
        return;
    }

    last_entropy = cur;

    float diff = cur - anchor;   /* change from last transition point */
    uint8_t hit;

    if (output == EYE_OPEN) {
        hit = (diff < -DELTA_THRESH) ? 1 : 0;   /* drop from anchor → closing */
    } else {
        hit = (diff >  DELTA_THRESH) ? 1 : 0;   /* rise from anchor → opening */
    }

    vote_buf[vote_idx] = hit;
    vote_idx = (vote_idx + 1) % WINDOW;

    if (sum_buf(vote_buf) >= VOTE_MIN) {
        output = (output == EYE_OPEN) ? EYE_CLOSED : EYE_OPEN;
        anchor = cur;                /* record new transition point */
        memset(vote_buf, 0, WINDOW); /* reset votes */
    }
}

EyeState EyeDetect_GetState(void)
{
    return output;
}
