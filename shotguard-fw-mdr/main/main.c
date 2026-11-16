/*
 * ShotGuard firmware - entry point.
 *
 *   app_main()  ->  bring up board, sensors, SD card  ->  start tasks
 *
 *   Core 1: acq task    (1 kHz: read FSR + IMU -> log -> detect touch/shot)
 *   Core 0: logger task (writes records to microSD, never blocks acq)
 *   Core 0: this task   (prints health stats every 2 s)
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "board.h"
#include "fsr.h"
#include "imu.h"
#include "detect.h"
#include "logger.h"
#include "acq.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ShotGuard fw %s", SG_FW_VERSION);

    ESP_ERROR_CHECK(board_init());
    ESP_ERROR_CHECK(fsr_init());

    imu_info_t imu = { 0 };
    if (imu_init(&imu) != ESP_OK) {
        /* Keep running so FSR work isn't blocked by a missing IMU */
        ESP_LOGE(TAG, "IMU init failed - IMU channels will read 0");
        imu.accel_lsb_per_g = 1024;
        imu.gyro_lsb_per_dps = 8.2f;
    }

    ESP_ERROR_CHECK(logger_init(&imu));
    detect_init(imu.accel_lsb_per_g);
    ESP_ERROR_CHECK(acq_start());

    /* Health monitor - this is how we verify spec S3 (>= 1 kHz, no misses) */
    acq_stats_t st;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        acq_get_stats(&st);
        ESP_LOGI(TAG, "samples=%lu overruns=%lu max_loop=%luus imu_err=%lu "
                      "touches=%lu shots=%lu dropped_records=%lu",
                 (unsigned long)st.samples, (unsigned long)st.overruns,
                 (unsigned long)st.max_loop_us, (unsigned long)st.imu_errors,
                 (unsigned long)st.touches, (unsigned long)st.shots,
                 (unsigned long)logger_dropped());
    }
}
