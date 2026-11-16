#include "board.h"
#include "config.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "board";
static i2c_master_bus_handle_t s_i2c_bus;

esp_err_t board_init(void)
{
    /* Status LED */
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << PIN_STATUS_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led));
    board_led_set(false);

#if !CONFIG_SHOTGUARD_SIM
    /* Feather: STEMMA QT port (our IMU) is unpowered until GPIO7 goes high.
     * Arduino/CircuitPython do this for you; in ESP-IDF we do it ourselves. */
    gpio_config_t pwr = {
        .pin_bit_mask = 1ULL << PIN_I2C_POWER,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&pwr));
    gpio_set_level(PIN_I2C_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(20));   /* let the IMU power up */
#endif

    /* Shared I2C master bus */
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* Feather also has 5k pull-ups */
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

#if CONFIG_SHOTGUARD_SIM
    ESP_LOGI(TAG, "board ready (Wokwi sim)");
#else
    ESP_LOGI(TAG, "board ready (Feather ESP32-S3)");
#endif
    return ESP_OK;
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_i2c_bus;
}

void board_led_set(bool on)
{
    gpio_set_level(PIN_STATUS_LED, on ? 1 : 0);
}
