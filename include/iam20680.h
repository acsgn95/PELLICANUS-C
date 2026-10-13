#ifndef IAM20680_H
#define IAM20680_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

/* SPI pins */
#define IAM20680_SPI_PORT   spi0
#define IAM20680_PIN_SCK    18
#define IAM20680_PIN_MOSI   19
#define IAM20680_PIN_MISO   20
#define IAM20680_PIN_CS     17
#define IAM20680_SPI_SPEED  1000000   /* 1 MHz */
/* The sensor's INT output (IMU_INT on the schematic) */
#define IAM20680_PIN_INT    16

/* Register map */
#define IAM20680_REG_SELF_TEST_X_GYRO   0x00
#define IAM20680_REG_SELF_TEST_Y_GYRO   0x01
#define IAM20680_REG_SELF_TEST_Z_GYRO   0x02
#define IAM20680_REG_SELF_TEST_X_ACCEL  0x0D
#define IAM20680_REG_SELF_TEST_Y_ACCEL  0x0E
#define IAM20680_REG_SELF_TEST_Z_ACCEL  0x0F
#define IAM20680_REG_SMPLRT_DIV         0x19
#define IAM20680_REG_CONFIG              0x1A
#define IAM20680_REG_GYRO_CONFIG         0x1B
#define IAM20680_REG_ACCEL_CONFIG        0x1C
#define IAM20680_REG_ACCEL_CONFIG2       0x1D
#define IAM20680_REG_INT_PIN_CFG         0x37
#define IAM20680_REG_INT_ENABLE          0x38
#define IAM20680_REG_INT_STATUS          0x3A
#define IAM20680_REG_ACCEL_XOUT_H        0x3B
#define IAM20680_REG_ACCEL_XOUT_L        0x3C
#define IAM20680_REG_ACCEL_YOUT_H        0x3D
#define IAM20680_REG_ACCEL_YOUT_L        0x3E
#define IAM20680_REG_ACCEL_ZOUT_H        0x3F
#define IAM20680_REG_ACCEL_ZOUT_L        0x40
#define IAM20680_REG_TEMP_OUT_H          0x41
#define IAM20680_REG_TEMP_OUT_L          0x42
#define IAM20680_REG_GYRO_XOUT_H         0x43
#define IAM20680_REG_GYRO_XOUT_L         0x44
#define IAM20680_REG_GYRO_YOUT_H         0x45
#define IAM20680_REG_GYRO_YOUT_L         0x46
#define IAM20680_REG_GYRO_ZOUT_H         0x47
#define IAM20680_REG_GYRO_ZOUT_L         0x48
#define IAM20680_REG_PWR_MGMT_1          0x6B
#define IAM20680_REG_PWR_MGMT_2          0x6C
#define IAM20680_REG_WHO_AM_I            0x75

#define IAM20680_WHO_AM_I_VAL   0xA9   /* IAM-20680 */
#define IAM20680HP_WHO_AM_I_VAL 0xF8   /* IAM-20680HP */
#define IAM20680HT_WHO_AM_I_VAL 0xFA   /* IAM-20680HT (automotive, high temperature) */

/* Accel full-scale range */
typedef enum {
    IAM20680_ACCEL_FS_2G  = 0x00,
    IAM20680_ACCEL_FS_4G  = 0x08,
    IAM20680_ACCEL_FS_8G  = 0x10,
    IAM20680_ACCEL_FS_16G = 0x18,
} iam20680_accel_fs_t;

/* Gyro full-scale range */
typedef enum {
    IAM20680_GYRO_FS_250DPS  = 0x00,
    IAM20680_GYRO_FS_500DPS  = 0x08,
    IAM20680_GYRO_FS_1000DPS = 0x10,
    IAM20680_GYRO_FS_2000DPS = 0x18,
} iam20680_gyro_fs_t;

/* In the sensor's own axes (right-handed, as its datasheet draws them); turning them into the
 * vehicle's is the host's calibration. */
typedef struct {
    float accel_x, accel_y, accel_z;   /* m/s^2 */
    float gyro_x,  gyro_y,  gyro_z;    /* rad/s */
    float temp;                          /* Celsius */
} iam20680_data_t;

/* One sample as the sensor gives it: accelerometer x/y/z, temperature, gyro x/y/z. */
typedef struct {
    int16_t v[7];
} iam20680_raw_t;

bool iam20680_init(iam20680_accel_fs_t accel_fs, iam20680_gyro_fs_t gyro_fs);
bool iam20680_read(iam20680_data_t *out);

/* Stream mode: a sample every 1 ms (1 kHz) through the digital low-pass filters given (the
 * CONFIG DLPF_CFG and ACCEL_CONFIG2 A_DLPF_CFG codes), with INT raised as each one is ready and
 * held until any register is read. */
void iam20680_start_stream(uint8_t gyro_dlpf_cfg, uint8_t accel_dlpf_cfg);
/* The latest sample's registers in one burst; reading them clears INT. */
void iam20680_read_raw(iam20680_raw_t *raw);
/* The mean of n raw samples, from their sums, in the units of iam20680_data_t. */
void iam20680_convert(const int32_t sum[7], int n, iam20680_data_t *out);

/* The WHO_AM_I value read by the last iam20680_init(), for diagnostics. */
uint8_t iam20680_who_am_i(void);
/* The part found by the last iam20680_init() ("IAM-20680HT"), or "unknown". */
const char *iam20680_part(void);

#endif /* IAM20680_H */
