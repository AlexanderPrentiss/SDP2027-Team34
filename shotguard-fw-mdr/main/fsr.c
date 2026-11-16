/*
 * fsr.c - FSR sampling with the ESP-IDF ADC "oneshot" driver.
 *
 * Oneshot = the CPU asks for one conversion per call. Simple and good enough
 * to start. If 4 reads + the IMU read don't fit in 1 ms, switch to the
 * "continuous" driver (esp_adc/adc_continuous.h), which uses DMA to sample
 * all four channels in hardware - see README "Next steps".
 */
#include "fsr.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "fsr";

static const int s_pins[SG_NUM_FSR] = {
    [ZONE_INSTEP]  = PIN_FSR_INSTEP,
    [ZONE_LACES]   = PIN_FSR_LACES,
    [ZONE_OUTSIDE] = PIN_FSR_OUTSIDE,
    [ZONE_TOE]     = PIN_FSR_TOE,
};

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_chan[SG_NUM_FSR];
static adc_cali_handle_t s_cali[SG_NUM_FSR];

esp_err_t fsr_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,      /* ~0-3.1 V input range on the S3 */
        .bitwidth = ADC_BITWIDTH_12,   /* 0-4095 */
    };

    for (int i = 0; i < SG_NUM_FSR; i++) {
        adc_unit_t unit;
        /* Map GPIO -> ADC channel so config.h only needs GPIO numbers */
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(s_pins[i], &unit, &s_chan[i]));
        if (unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "GPIO%d is not on ADC1", s_pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_chan[i], &chan_cfg));

        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = s_chan[i],
            .atten = chan_cfg.atten,
            .bitwidth = chan_cfg.bitwidth,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali[i]) != ESP_OK) {
            s_cali[i] = NULL;  /* e.g. in the simulator: fall back to linear */
        }
        ESP_LOGI(TAG, "zone %d -> GPIO%d (ADC1_CH%d)", i, s_pins[i], s_chan[i]);
    }
    return ESP_OK;
}

esp_err_t fsr_read_raw(uint16_t out[SG_NUM_FSR])
{
    for (int i = 0; i < SG_NUM_FSR; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, s_chan[i], &raw);
        if (err != ESP_OK) {
            return err;
        }
        out[i] = (uint16_t)raw;
    }
    return ESP_OK;
}

int fsr_raw_to_mv(int zone, uint16_t raw)
{
    int mv = 0;
    if (zone >= 0 && zone < SG_NUM_FSR && s_cali[zone] &&
        adc_cali_raw_to_voltage(s_cali[zone], raw, &mv) == ESP_OK) {
        return mv;
    }
    return (int)raw * 3100 / 4095;  /* rough uncalibrated estimate */
}
