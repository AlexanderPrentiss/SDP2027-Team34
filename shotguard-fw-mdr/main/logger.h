/*
 * logger.h - Block 4 (device side): microSD logging.
 *
 * The acquisition task must never wait on the SD card (a single write can
 * stall for tens of ms). So records go through a pool of buffers:
 *
 *   acq task:    r = logger_get_free(); fill r; logger_submit(r);
 *   logger task: takes r, writes it to the card, returns it to the pool.
 *
 * If the SD card is missing (e.g. in the simulator), records are printed to
 * the serial console instead so the pipeline can still be tested.
 */
#pragma once

#include "esp_err.h"
#include "log_format.h"
#include "imu.h"
#include "config.h"

/* One record = header + up to 100 samples (~2.8 kB). DATA records fill
 * samples[]; EVENT records leave it empty (n_samples = 0). */
typedef struct {
    sg_record_header_t hdr;
    sg_sample_t samples[SG_BLOCK_SAMPLES];
} sg_record_t;

esp_err_t logger_init(const imu_info_t *imu);

/* Non-blocking. Returns NULL if every buffer is in use (record is dropped
 * and counted - the SD card is falling behind). */
sg_record_t *logger_get_free(void);
void logger_submit(sg_record_t *r);

uint32_t logger_dropped(void);
