/*
 * imu.c - minimal register-level IMU driver (no Arduino library needed).
 *
 * How talking to an I2C sensor works:
 *   write [reg]        -> select a register
 *   read  N bytes      -> sensor returns reg, reg+1, ... (auto-increment)
 * i2c_master_transmit_receive() does both in one transaction.
 *
 * ICM-20649 specifics (see the datasheet register map):
 *   - Registers are split into 4 banks; REG_BANK_SEL (0x7F) picks one.
 *   - Sensor data lives in bank 0 starting at ACCEL_XOUT_H (0x2D):
 *     ax ay az gx gy gz, 16-bit big-endian, 12 bytes total.
 *   - Range / filter / rate config lives in bank 2.
 */
#include "imu.h"
#include "board.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "imu";
static i2c_master_dev_handle_t s_dev;

#define I2C_TIMEOUT_MS  5

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, len, I2C_TIMEOUT_MS);
}

static esp_err_t attach(uint8_t addr)
{
    i2c_master_dev_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = SG_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(board_i2c_bus(), &dev_cfg, &s_dev);
}

/* Probe a list of candidate addresses, attach to the first that ACKs. */
static esp_err_t find_device(const uint8_t *addrs, int n)
{
    for (int i = 0; i < n; i++) {
        if (i2c_master_probe(board_i2c_bus(), addrs[i], 50) == ESP_OK) {
            ESP_LOGI(TAG, "found device at 0x%02X", addrs[i]);
            return attach(addrs[i]);
        }
    }
    ESP_LOGE(TAG, "no IMU found on I2C bus - check wiring / STEMMA power");
    return ESP_ERR_NOT_FOUND;
}

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

#if !CONFIG_SHOTGUARD_SIM
/* ======================= ICM-20649 (real hardware) ======================= */

#define ICM_REG_BANK_SEL        0x7F
/* bank 0 */
#define ICM_WHO_AM_I            0x00
#define ICM_PWR_MGMT_1          0x06
#define ICM_PWR_MGMT_2          0x07
#define ICM_ACCEL_XOUT_H        0x2D
/* bank 2 */
#define ICM_GYRO_SMPLRT_DIV     0x00
#define ICM_GYRO_CONFIG_1       0x01
#define ICM_ACCEL_SMPLRT_DIV_1  0x10
#define ICM_ACCEL_SMPLRT_DIV_2  0x11
#define ICM_ACCEL_CONFIG        0x14

#define ICM20649_WHO_AM_I_VAL   0xE1

/* Full-scale selects (2-bit FS_SEL field, bits [2:1]) */
#define ICM_ACCEL_FS_30G        3     /* 1024 LSB/g */
#define ICM_GYRO_FS_4000DPS     3     /* 8.2 LSB/dps */
/* DLPF config (bits [5:3]) + FCHOICE (bit 0). Sample rate with DLPF on is
 * 1125 Hz / (1 + SMPLRT_DIV), so DIV = 0 -> 1125 Hz, just above our 1 kHz.
 * TODO(IMU owner): pick DLPF bandwidth from the datasheet tables. */
#define ICM_DLPF_CFG            0

static esp_err_t bank(uint8_t b)
{
    return reg_write(ICM_REG_BANK_SEL, (uint8_t)(b << 4));
}

esp_err_t imu_init(imu_info_t *info)
{
    /* Adafruit breakout default is 0x69; 0x68 if the AD0 jumper is changed */
    static const uint8_t addrs[] = { 0x69, 0x68 };
    esp_err_t err = find_device(addrs, 2);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(bank(0));
    ESP_ERROR_CHECK(reg_write(ICM_PWR_MGMT_1, 0x80));       /* device reset */
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(bank(0));
    ESP_ERROR_CHECK(reg_write(ICM_PWR_MGMT_1, 0x01));       /* wake, auto clock */
    ESP_ERROR_CHECK(reg_write(ICM_PWR_MGMT_2, 0x00));       /* accel+gyro on */
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t who = 0;
    ESP_ERROR_CHECK(reg_read(ICM_WHO_AM_I, &who, 1));
    if (who != ICM20649_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X, expected 0x%02X", who, ICM20649_WHO_AM_I_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_ERROR_CHECK(bank(2));
    ESP_ERROR_CHECK(reg_write(ICM_GYRO_SMPLRT_DIV, 0));
    ESP_ERROR_CHECK(reg_write(ICM_GYRO_CONFIG_1,
                              (ICM_DLPF_CFG << 3) | (ICM_GYRO_FS_4000DPS << 1) | 1));
    ESP_ERROR_CHECK(reg_write(ICM_ACCEL_SMPLRT_DIV_1, 0));
    ESP_ERROR_CHECK(reg_write(ICM_ACCEL_SMPLRT_DIV_2, 0));
    ESP_ERROR_CHECK(reg_write(ICM_ACCEL_CONFIG,
                              (ICM_DLPF_CFG << 3) | (ICM_ACCEL_FS_30G << 1) | 1));
    ESP_ERROR_CHECK(bank(0));                               /* data regs live here */

    info->type = SG_IMU_ICM20649;
    info->accel_lsb_per_g = 1024;
    info->gyro_lsb_per_dps = 8.2f;
    ESP_LOGI(TAG, "ICM-20649 ready: +/-30 g, +/-4000 dps, 1125 Hz");
    return ESP_OK;
}

esp_err_t imu_read(int16_t accel[3], int16_t gyro[3])
{
    uint8_t b[12];
    esp_err_t err = reg_read(ICM_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;
    for (int i = 0; i < 3; i++) {
        accel[i] = be16(&b[2 * i]);
        gyro[i]  = be16(&b[6 + 2 * i]);
    }
    return ESP_OK;
}

#else
/* ======================= MPU6050 (Wokwi simulator) ======================= */

#define MPU_PWR_MGMT_1   0x6B
#define MPU_WHO_AM_I     0x75
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B   /* ax ay az temp gx gy gz = 14 bytes */

esp_err_t imu_init(imu_info_t *info)
{
    static const uint8_t addrs[] = { 0x68, 0x69 };
    esp_err_t err = find_device(addrs, 2);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(reg_write(MPU_PWR_MGMT_1, 0x00));   /* wake */
    uint8_t who = 0;
    ESP_ERROR_CHECK(reg_read(MPU_WHO_AM_I, &who, 1));
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X", who);
    ESP_ERROR_CHECK(reg_write(MPU_ACCEL_CONFIG, 3 << 3)); /* +/-16 g   */
    ESP_ERROR_CHECK(reg_write(MPU_GYRO_CONFIG, 3 << 3));  /* +/-2000 dps */

    info->type = SG_IMU_MPU6050;
    info->accel_lsb_per_g = 2048;
    info->gyro_lsb_per_dps = 16.4f;
    return ESP_OK;
}

esp_err_t imu_read(int16_t accel[3], int16_t gyro[3])
{
    uint8_t b[14];
    esp_err_t err = reg_read(MPU_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;
    for (int i = 0; i < 3; i++) {
        accel[i] = be16(&b[2 * i]);
        gyro[i]  = be16(&b[8 + 2 * i]);   /* skip 2 temp bytes */
    }
    return ESP_OK;
}
#endif
