#ifndef PELLICANUS_H
#define PELLICANUS_H

/* I2C1 shared bus pins (MMC5983MA + MS5637) */
#define PELLICANUS_I2C_PORT  i2c1
#define PELLICANUS_PIN_SDA   26
#define PELLICANUS_PIN_SCL   27
#define PELLICANUS_I2C_SPEED 400000   /* 400 kHz: 1 kOhm pull-ups on the board */

/* IMU: the sensor samples at 1 kHz; each output sample is the mean of PELLICANUS_IMU_DECIMATION
 * of them (250 Hz). The sensor's low-pass filters are set well below the output's Nyquist
 * frequency (125 Hz): gyro DLPF_CFG 3 (41 Hz), accelerometer A_DLPF_CFG 3 (44.8 Hz). */
#define PELLICANUS_IMU_SENSOR_HZ  1000
#define PELLICANUS_IMU_DECIMATION 4
#define PELLICANUS_IMU_HZ         (PELLICANUS_IMU_SENSOR_HZ / PELLICANUS_IMU_DECIMATION)
#define PELLICANUS_IMU_GYRO_DLPF  3
#define PELLICANUS_IMU_ACCEL_DLPF 3
/* GNSS update rate */
#define PELLICANUS_GNSS_HZ   4

/* Reported in every STATUS packet. */
#define PELLICANUS_FIRMWARE_VERSION "2.1.0"

/* IMU full-scale ranges. A vehicle's vibration and manoeuvres clip +-2 g and +-500 deg/s; the
 * widest ranges cost resolution the sensor's noise already hides. */
#define PELLICANUS_ACCEL_FS  IAM20680_ACCEL_FS_16G
#define PELLICANUS_GYRO_FS   IAM20680_GYRO_FS_2000DPS
/* The same ranges in units, for the STATUS packet: keep them with the two above. */
#define PELLICANUS_ACCEL_RANGE_G   16
#define PELLICANUS_GYRO_RANGE_DPS  2000

#include "iam20680.h"
#include "mmc5983ma.h"
#include "ms5637.h"
#include "sl871_gnss.h"

#endif /* PELLICANUS_H */
