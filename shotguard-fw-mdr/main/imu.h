/*
 * imu.h - Block 2: shin IMU over I2C.
 * Hardware: ICM-20649 (+/-30 g, +/-4000 dps). Simulator: MPU6050.
 * Everything above this header is identical for both.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "log_format.h"

typedef struct {
    sg_imu_type_t type;
    uint16_t accel_lsb_per_g;
    float    gyro_lsb_per_dps;
} imu_info_t;

esp_err_t imu_init(imu_info_t *info_out);

/* One burst read of accel xyz + gyro xyz, raw counts. */
esp_err_t imu_read(int16_t accel[3], int16_t gyro[3]);
