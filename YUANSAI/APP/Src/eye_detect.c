#include "eye_detect.h"
#include <string.h>
#include "main.h"

#define DELTA_THRESH  0.15f
#define EDGE_RATIO    0.05f     /* edge threshold = anchor * ratio */
#define WINDOW        7
#define VOTE_MIN      4

static EyeState output = EYE_OPEN;
static float    last_entropy = 0.0f;
static float    last_edge = 0.0f;
static float    anchor_e = 0.0f;     /* entropy at last transition */
static float    anchor_ed = 0.0f;    /* edge energy at last transition */
static uint8_t  vote_buf[WINDOW];
static uint8_t  vote_idx = 0;
static uint8_t  initialized = 0;

void EyeDetect_Init(void)
{
    output = EYE_OPEN;
    last_entropy = 0.0f;
    last_edge = 0.0f;
    anchor_e = 0.0f;
    anchor_ed = 0.0f;
    memset(vote_buf, 0, WINDOW);
    vote_idx = 0;
    initialized = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

static uint8_t sum_buf(const uint8_t *buf)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < WINDOW; i++) s += buf[i];
    return s;
}

void EyeDetect_Update(float entropy, float edge)
{
    if (!initialized) {
        last_entropy = entropy;
        last_edge = edge;
        anchor_e = entropy;
        anchor_ed = edge;
        initialized = 1;
        return;
    }

    last_entropy = entropy;
    last_edge = edge;

    float diff_e = entropy - anchor_e;
    float diff_ed = edge - anchor_ed;
    float edge_thresh = anchor_ed * EDGE_RATIO;  /* proportional to value */
    uint8_t hit;

    if (output == EYE_OPEN) {
        /* both entropy drop AND edge drop → closing */
        hit = (diff_e < -DELTA_THRESH && diff_ed < -edge_thresh) ? 1 : 0;
    } else {
        /* both entropy rise AND edge rise → opening */
        hit = (diff_e >  DELTA_THRESH && diff_ed >  edge_thresh) ? 1 : 0;
    }

    vote_buf[vote_idx] = hit;
    vote_idx = (vote_idx + 1) % WINDOW;

    if (sum_buf(vote_buf) >= VOTE_MIN) {
        output = (output == EYE_OPEN) ? EYE_CLOSED : EYE_OPEN;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
            (output == EYE_CLOSED) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        anchor_e = entropy;
        anchor_ed = edge;
        memset(vote_buf, 0, WINDOW);
    }
}

EyeState EyeDetect_GetState(void)
{
    return output;
}
