/*
 * log_format.h - the binary file format on the microSD card.
 *
 * This header is the CONTRACT between the firmware and the laptop app.
 * Any change here must bump SG_FORMAT_VERSION and be mirrored in
 * tools/sg_decode.py. All fields are little-endian, structs are packed
 * (no padding bytes), so Python can read them with struct.unpack.
 *
 * File layout:
 *   sg_file_header_t                         once, at the start
 *   then repeated until end of file:
 *     sg_record_header_t + sg_sample_t[n]    SG_REC_DATA  (n = 100, i.e. 100 ms)
 *     sg_record_header_t                     SG_REC_EVENT (n = 0, a detected touch/shot)
 */
#pragma once

#include <stdint.h>

#define SG_FILE_MAGIC       0x474C4753u   /* bytes "SGLG" */
#define SG_FORMAT_VERSION   2
#define SG_RECORD_SYNC      0xA55Au

typedef enum {
    SG_IMU_ICM20649 = 0,
    SG_IMU_MPU6050  = 1,   /* Wokwi simulator only */
} sg_imu_type_t;

typedef enum {
    SG_REC_DATA  = 1,      /* block of raw samples                */
    SG_REC_EVENT = 2,      /* detector output, no samples         */
} sg_record_type_t;

typedef enum {
    SG_EVENT_NONE  = 0,
    SG_EVENT_TOUCH = 1,
    SG_EVENT_SHOT  = 2,
} sg_event_kind_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;            /* SG_FILE_MAGIC                               */
    uint16_t version;          /* SG_FORMAT_VERSION                           */
    uint16_t sample_rate_hz;
    uint8_t  imu_type;         /* sg_imu_type_t                               */
    uint8_t  n_fsr;
    uint16_t accel_lsb_per_g;  /* raw accel / this = g                        */
    float    gyro_lsb_per_dps; /* raw gyro  / this = deg/s                    */
    uint16_t adc_max;          /* 4095 for 12-bit                             */
    uint16_t reserved;
    char     fw_version[16];
} sg_file_header_t;

typedef struct __attribute__((packed)) {
    uint64_t t_us;             /* microseconds since boot                     */
    uint16_t fsr[4];           /* raw ADC counts: instep, laces, outside, toe */
    int16_t  accel[3];         /* raw IMU counts x, y, z                      */
    int16_t  gyro[3];
} sg_sample_t;                 /* 28 B -> 28 kB/s -> ~200 MB per 2 h          */

typedef struct __attribute__((packed)) {
    uint16_t sync;             /* SG_RECORD_SYNC: lets the app resync after corruption */
    uint8_t  type;             /* sg_record_type_t                            */
    uint8_t  kind;             /* EVENT: sg_event_kind_t                      */
    uint32_t seq;              /* DATA: block counter, EVENT: event counter   */
    uint16_t n_samples;        /* number of sg_sample_t that follow           */
    uint8_t  zone;             /* EVENT: FSR zone with the highest reading    */
    uint8_t  peak_accel_dg;    /* EVENT: peak |a| in tenths of g (255 = 25.5 g+) */
    uint64_t event_t_us;       /* EVENT: time of contact; DATA: time of 1st sample */
} sg_record_header_t;

_Static_assert(sizeof(sg_sample_t) == 28, "sample layout changed - bump version");
_Static_assert(sizeof(sg_record_header_t) == 20, "record layout changed - bump version");
_Static_assert(sizeof(sg_file_header_t) == 36, "file header changed - bump version");
