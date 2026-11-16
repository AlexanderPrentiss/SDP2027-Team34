#include "logger.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

static const char *TAG = "logger";

static sg_record_t   s_pool[SG_LOG_POOL_SIZE];
static QueueHandle_t s_free_q;      /* empty buffers   */
static QueueHandle_t s_full_q;      /* buffers to write */
static FILE         *s_file;        /* NULL -> serial fallback */
static volatile uint32_t s_dropped;

/* ---------- SD card ---------- */

static esp_err_t sd_mount(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SD_MOSI,
        .miso_io_num = PIN_SD_MISO,
        .sclk_io_num = PIN_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) return err;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = PIN_SD_CS;
    slot.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,   /* never wipe a card silently */
        .max_files = 2,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *card;
    err = esp_vfs_fat_sdspi_mount(SG_SD_MOUNT_POINT, &host, &slot, &mount, &card);
    if (err == ESP_OK) sdmmc_card_print_info(stdout, card);
    return err;
}

/* Next unused name: /sdcard/SG0001.BIN, SG0002.BIN, ... (8.3 names) */
static FILE *open_session_file(void)
{
    char path[32];
    struct stat st;
    for (int i = 1; i < 10000; i++) {
        snprintf(path, sizeof(path), SG_SD_MOUNT_POINT "/SG%04d.BIN", i);
        if (stat(path, &st) != 0) {
            ESP_LOGI(TAG, "logging to %s", path);
            return fopen(path, "wb");
        }
    }
    return NULL;
}

/* ---------- serial fallback (simulator / no card) ---------- */

static const char *const KIND[] = { "none", "TOUCH", "SHOT" };
static const char *const ZONE[] = { "instep", "laces", "outside", "toe" };

static void print_record(const sg_record_t *r)
{
    if (r->hdr.type == SG_REC_EVENT) {
        printf(">>> %s #%lu  zone=%s  peak=%.1f g  t=%llu us\n",
               KIND[r->hdr.kind % 3], (unsigned long)r->hdr.seq,
               ZONE[r->hdr.zone % 4], r->hdr.peak_accel_dg / 10.0,
               (unsigned long long)r->hdr.event_t_us);
    } else if (r->hdr.seq % 10 == 0) {
        /* 1 kHz is far too fast for a serial console; show 1 sample per second */
        const sg_sample_t *s = &r->samples[0];
        printf("data blk=%lu fsr=[%u %u %u %u] acc=[%d %d %d] gyr=[%d %d %d]\n",
               (unsigned long)r->hdr.seq,
               s->fsr[0], s->fsr[1], s->fsr[2], s->fsr[3],
               s->accel[0], s->accel[1], s->accel[2],
               s->gyro[0], s->gyro[1], s->gyro[2]);
    }
}

/* ---------- task ---------- */

static void logger_task(void *arg)
{
    uint32_t since_sync = 0;
    sg_record_t *r;
    for (;;) {
        xQueueReceive(s_full_q, &r, portMAX_DELAY);

        if (s_file) {
            size_t n = r->hdr.n_samples;
            if (fwrite(&r->hdr, sizeof(r->hdr), 1, s_file) != 1 ||
                fwrite(r->samples, sizeof(sg_sample_t), n, s_file) != n) {
                ESP_LOGE(TAG, "write failed - card full or removed?");
            }
            /* Flush periodically so a dead battery loses <= ~1 s of data */
            if (++since_sync >= SG_FSYNC_EVERY_RECORDS) {
                fflush(s_file);
                fsync(fileno(s_file));
                since_sync = 0;
            }
        } else {
            print_record(r);
        }

        xQueueSend(s_free_q, &r, 0);
    }
}

esp_err_t logger_init(const imu_info_t *imu)
{
    s_free_q = xQueueCreate(SG_LOG_POOL_SIZE, sizeof(sg_record_t *));
    s_full_q = xQueueCreate(SG_LOG_POOL_SIZE, sizeof(sg_record_t *));
    for (int i = 0; i < SG_LOG_POOL_SIZE; i++) {
        sg_record_t *p = &s_pool[i];
        xQueueSend(s_free_q, &p, 0);
    }

    esp_err_t err = sd_mount();
    if (err == ESP_OK) {
        s_file = open_session_file();
    } else {
        ESP_LOGW(TAG, "SD mount failed (%s) - logging to serial instead",
                 esp_err_to_name(err));
    }

    if (s_file) {
        sg_file_header_t h = {
            .magic = SG_FILE_MAGIC,
            .version = SG_FORMAT_VERSION,
            .sample_rate_hz = SG_SAMPLE_RATE_HZ,
            .imu_type = imu->type,
            .n_fsr = SG_NUM_FSR,
            .accel_lsb_per_g = imu->accel_lsb_per_g,
            .gyro_lsb_per_dps = imu->gyro_lsb_per_dps,
            .adc_max = 4095,
        };
        strncpy(h.fw_version, SG_FW_VERSION, sizeof(h.fw_version) - 1);
        fwrite(&h, sizeof(h), 1, s_file);
        fflush(s_file);
    }

    /* Core 0, below the acquisition task's priority */
    xTaskCreatePinnedToCore(logger_task, "logger", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

sg_record_t *logger_get_free(void)
{
    sg_record_t *r = NULL;
    if (xQueueReceive(s_free_q, &r, 0) != pdTRUE) {
        s_dropped++;
        return NULL;
    }
    return r;
}

void logger_submit(sg_record_t *r)
{
    xQueueSend(s_full_q, &r, 0);   /* can't fail: pool size == queue size */
}

uint32_t logger_dropped(void)
{
    return s_dropped;
}
