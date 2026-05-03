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

#define IAM20680_WHO_AM_I_VAL   0xA9

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

typedef struct {
    float accel_x, accel_y, accel_z;   /* m/s^2 */
    float gyro_x,  gyro_y,  gyro_z;    /* deg/s */
    float temp;                          /* Celsius */
} iam20680_data_t;

bool iam20680_init(iam20680_accel_fs_t accel_fs, iam20680_gyro_fs_t gyro_fs);
bool iam20680_read(iam20680_data_t *out);

#endif /* IAM20680_H */
