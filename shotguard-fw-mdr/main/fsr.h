/*
 * fsr.h - Block 1: pressure sensing. Four FSR voltage dividers on ADC1.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "config.h"

esp_err_t fsr_init(void);

/* Read all four zones as raw 12-bit counts, ordered by sg_zone_t.
 * Called once per sample from the acquisition task. */
esp_err_t fsr_read_raw(uint16_t out[SG_NUM_FSR]);

/* Convert a raw count to millivolts using the chip's factory calibration.
 * For debugging/calibration only - the log stores raw counts. */
int fsr_raw_to_mv(int zone, uint16_t raw);
