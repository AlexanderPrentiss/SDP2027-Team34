/*
 * acq.c - the real-time heart of the firmware.
 *
 * A hardware timer (GPTimer) interrupts every 1 ms. The interrupt does almost
 * nothing - it just wakes acq_task. acq_task then:
 *   1. reads the 4 FSRs and the IMU into one timestamped sample
 *   2. appends it to the current 100-sample block; full blocks go to the logger
 *   3. runs the detector; touches/shots go to the logger as EVENT records
 * acq_task never waits on the SD card - that's the logger task's job.
 */
#include "acq.h"
#include "config.h"
#include "fsr.h"
#include "imu.h"
#include "detect.h"
#include "logger.h"
#include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "acq";

static TaskHandle_t s_task;
static acq_stats_t  s_stats;

/* ---- 1 kHz hardware timer interrupt: wake the task, nothing else ---- */
static bool on_timer_alarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *e,
                           void *ctx)
{
    BaseType_t woke = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woke);
    return woke == pdTRUE;   /* switch to acq_task right away if it should run */
}

static esp_err_t start_timer(void)
{
    gptimer_handle_t timer;
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,             /* counts microseconds */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));

    gptimer_alarm_config_t alarm = {
        .alarm_count = SG_SAMPLE_PERIOD_US,   /* fire every 1000 us ... */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,   /* ... forever */
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    return gptimer_start(timer);
}

static void acq_task(void *arg)
{
    sg_record_t *block = NULL;   /* 100-sample record being filled */
    uint32_t block_seq = 0;
    uint32_t event_seq = 0;
    uint32_t led_off_at = 0;

    for (;;) {
        /* Sleep until the timer fires. A count > 1 means we missed ticks. */
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        if (ticks == 0) continue;
        if (ticks > 1) s_stats.overruns += ticks - 1;

        int64_t t0 = esp_timer_get_time();

        /* ---- 1. acquire one sample ---- */
        sg_sample_t s = { .t_us = (uint64_t)t0 };
        if (fsr_read_raw(s.fsr) != ESP_OK) s_stats.adc_errors++;
        if (imu_read(s.accel, s.gyro) != ESP_OK) s_stats.imu_errors++;
        s_stats.samples++;

        /* ---- 2. log it (raw data, always) ---- */
        if (!block) {
            block = logger_get_free();          /* NULL if SD is falling behind */
            if (block) {
                block->hdr = (sg_record_header_t){
                    .sync = SG_RECORD_SYNC, .type = SG_REC_DATA,
                    .seq = block_seq++, .event_t_us = s.t_us,
                };
            }
        }
        if (block) {
            block->samples[block->hdr.n_samples++] = s;
            if (block->hdr.n_samples == SG_BLOCK_SAMPLES) {
                logger_submit(block);
                block = NULL;
            }
        }

        /* ---- 3. detect touch / shot ---- */
        detect_result_t d = detect_process(&s);
        if (d.kind != SG_EVENT_NONE) {
            if (d.kind == SG_EVENT_SHOT) s_stats.shots++;
            else                         s_stats.touches++;
            board_led_set(true);
            led_off_at = s_stats.samples + 200;   /* 200 ms blink */

            sg_record_t *ev = logger_get_free();
            if (ev) {
                uint32_t dg = d.peak_accel_mg / 100;
                ev->hdr = (sg_record_header_t){
                    .sync = SG_RECORD_SYNC, .type = SG_REC_EVENT,
                    .kind = d.kind, .seq = event_seq, .n_samples = 0,
                    .zone = d.zone, .peak_accel_dg = dg > 255 ? 255 : dg,
                    .event_t_us = s.t_us,
                };
                logger_submit(ev);
            }
            event_seq++;
        }
        if (led_off_at && s_stats.samples >= led_off_at) {
            board_led_set(false);
            led_off_at = 0;
        }

        uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
        if (dt > s_stats.max_loop_us) s_stats.max_loop_us = dt;
    }
}

esp_err_t acq_start(void)
{
    /* Core 1, high priority: nothing else may delay sampling */
    BaseType_t ok = xTaskCreatePinnedToCore(acq_task, "acq", 4096, NULL,
                                            configMAX_PRIORITIES - 2, &s_task, 1);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "sampling at %d Hz", SG_SAMPLE_RATE_HZ);
    return start_timer();
}

void acq_get_stats(acq_stats_t *out)
{
    *out = s_stats;   /* fine for a debug printout */
}
