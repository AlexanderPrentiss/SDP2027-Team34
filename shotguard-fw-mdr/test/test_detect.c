/*
 * Host-side unit test for detect.c - runs on any laptop, no ESP32 needed:
 *   gcc -std=c11 -Wall -Itest -Imain main/detect.c test/test_detect.c -lm -o test/test_detect
 *   ./test/test_detect
 * Each block below is one scenario the detector must get right.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "detect.h"
#include "config.h"

#define LSB_PER_G 1024   /* ICM-20649 at +/-30 g */

static sg_sample_t sample(float accel_g, uint16_t toe_fsr)
{
    sg_sample_t s; memset(&s, 0, sizeof s);
    s.accel[0] = (int16_t)(accel_g * LSB_PER_G);
    for (int i = 0; i < 4; i++) s.fsr[i] = 100;
    s.fsr[ZONE_TOE] = toe_fsr;
    return s;
}

/* feed n identical samples, return the last non-NONE result (or NONE) */
static detect_result_t feed(sg_sample_t s, int n)
{
    detect_result_t last = { .kind = SG_EVENT_NONE };
    for (int i = 0; i < n; i++) {
        detect_result_t r = detect_process(&s);
        if (r.kind != SG_EVENT_NONE) last = r;
    }
    return last;
}

static void rest(void) { feed(sample(1.0f, 100), DETECT_REFRACTORY_MS + DETECT_ACCEL_WINDOW_MS + 10); }

int main(void)
{
    detect_init(LSB_PER_G);

    /* standing still -> nothing */
    assert(feed(sample(1.0f, 100), 1000).kind == SG_EVENT_NONE);

    /* step: foot loads the FSR with no leg swing -> ignored */
    assert(feed(sample(1.2f, 3000), 50).kind == SG_EVENT_NONE);
    rest();

    /* touch: moderate swing (4 g), then contact 10 ms later */
    feed(sample(4.0f, 100), 5);
    feed(sample(1.0f, 100), 10);
    detect_result_t t = feed(sample(1.0f, 3000), 1);
    assert(t.kind == SG_EVENT_TOUCH && t.zone == ZONE_TOE);
    rest();

    /* shot: hard swing (15 g), then contact */
    feed(sample(15.0f, 100), 5);
    detect_result_t k = feed(sample(1.0f, 3000), 1);
    assert(k.kind == SG_EVENT_SHOT && k.peak_accel_mg >= 14900);

    /* a second contact inside the refractory period -> ignored */
    feed(sample(1.0f, 100), 5);
    feed(sample(15.0f, 100), 5);
    assert(feed(sample(1.0f, 3000), 1).kind == SG_EVENT_NONE);
    rest();

    /* swing too long before contact (outside the window) -> ignored */
    feed(sample(15.0f, 100), 5);
    feed(sample(1.0f, 100), DETECT_ACCEL_WINDOW_MS + 5);
    assert(feed(sample(1.0f, 3000), 1).kind == SG_EVENT_NONE);

    printf("all detector tests passed\n");
    return 0;
}
