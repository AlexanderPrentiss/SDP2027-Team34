/*
 * detect.c - threshold detector.
 *
 * Every sample:
 *   1. |a| = sqrt(ax^2 + ay^2 + az^2) in milli-g.
 *   2. "Peak hold": remember the largest |a| seen in the last
 *      DETECT_ACCEL_WINDOW_MS. A kick swing happens just BEFORE the foot
 *      meets the ball, so we look backwards from the moment of contact.
 *   3. Contact = the largest FSR crosses DETECT_FSR_THRESH_COUNTS going up
 *      (rising edge, so a foot that stays planted only counts once).
 *   4. On contact, classify by the held peak: SHOT / TOUCH / ignore.
 *   5. Refractory period so one ball contact = one event.
 */
#include "detect.h"
#include "config.h"
#include <math.h>

#define MS_TO_SAMPLES(ms) ((ms) * SG_SAMPLE_RATE_HZ / 1000)

static float    s_mg_per_lsb;
static uint32_t s_peak_mg;        /* held peak |a| */
static uint32_t s_peak_age;       /* samples since the peak was set */
static int      s_fsr_was_high;
static uint32_t s_refractory;

void detect_init(uint16_t accel_lsb_per_g)
{
    s_mg_per_lsb = 1000.0f / (float)accel_lsb_per_g;
    s_peak_mg = 0;
    s_peak_age = 0;
    s_fsr_was_high = 0;
    s_refractory = 0;
}

detect_result_t detect_process(const sg_sample_t *s)
{
    detect_result_t r = { .kind = SG_EVENT_NONE };

    /* 1. acceleration magnitude */
    float ax = s->accel[0], ay = s->accel[1], az = s->accel[2];
    uint32_t a_mg = (uint32_t)(sqrtf(ax * ax + ay * ay + az * az) * s_mg_per_lsb);

    /* 2. peak hold over the last window */
    if (a_mg >= s_peak_mg || s_peak_age >= MS_TO_SAMPLES(DETECT_ACCEL_WINDOW_MS)) {
        s_peak_mg = a_mg;
        s_peak_age = 0;
    } else {
        s_peak_age++;
    }

    /* 3. strongest FSR + rising edge */
    uint16_t fsr_max = 0;
    uint8_t zone = 0;
    for (uint8_t i = 0; i < SG_NUM_FSR; i++) {
        if (s->fsr[i] > fsr_max) {
            fsr_max = s->fsr[i];
            zone = i;
        }
    }
    int fsr_high = fsr_max >= DETECT_FSR_THRESH_COUNTS;
    int contact = fsr_high && !s_fsr_was_high;
    s_fsr_was_high = fsr_high;

    if (s_refractory > 0) {
        s_refractory--;
        return r;
    }
    if (!contact) {
        return r;
    }

    /* 4. classify */
    if (s_peak_mg >= DETECT_SHOT_ACCEL_MG) {
        r.kind = SG_EVENT_SHOT;
    } else if (s_peak_mg >= DETECT_TOUCH_ACCEL_MG) {
        r.kind = SG_EVENT_TOUCH;
    } else {
        return r;   /* a step - foot loaded without a swing */
    }

    /* 5. report + lock out */
    r.zone = zone;
    r.peak_accel_mg = s_peak_mg > 65535 ? 65535 : (uint16_t)s_peak_mg;
    s_refractory = MS_TO_SAMPLES(DETECT_REFRACTORY_MS);
    return r;
}
