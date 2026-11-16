/*
 * detect.h - Block 3: touch vs. shot detection.
 *
 * Pure logic: no ESP-IDF calls, no hardware. It only looks at samples you
 * hand it, which is why it can be unit-tested on a laptop (test/).
 */
#pragma once

#include <stdint.h>
#include "log_format.h"

typedef struct {
    sg_event_kind_t kind;      /* NONE, TOUCH or SHOT                        */
    uint8_t  zone;             /* sg_zone_t with the highest FSR at contact  */
    uint16_t peak_accel_mg;    /* peak |a| in the window before contact      */
} detect_result_t;

void detect_init(uint16_t accel_lsb_per_g);

/* Feed one sample (call 1000x per second). Returns kind == SG_EVENT_NONE
 * except on the single sample where a touch or shot is recognised. */
detect_result_t detect_process(const sg_sample_t *s);
