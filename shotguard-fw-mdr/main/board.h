/*
 * board.h - board bring-up: power rails, status LED, shared I2C bus.
 * Owner: hardware / PCB (pin changes for PCB v1 go in config.h + here).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t board_init(void);

/* Shared I2C bus handle. IMU and (later) the MAX17048 fuel gauge attach to it. */
i2c_master_bus_handle_t board_i2c_bus(void);

void board_led_set(bool on);
