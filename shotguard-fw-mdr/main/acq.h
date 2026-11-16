/*
 * acq.h - real-time acquisition: a hardware timer fires every 1 ms, the
 * acquisition task wakes, reads FSRs + IMU, timestamps the sample, runs the
 * detector, and hands records to the logger. Only main.c calls these.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t samples;       /* total samples taken                          */
    uint32_t overruns;      /* timer ticks missed (sampling took > 1 ms)    */
    uint32_t imu_errors;
    uint32_t adc_errors;
    uint32_t touches;
    uint32_t shots;
    uint32_t max_loop_us;   /* worst-case time to take one sample           */
} acq_stats_t;

esp_err_t acq_start(void);
void acq_get_stats(acq_stats_t *out);
