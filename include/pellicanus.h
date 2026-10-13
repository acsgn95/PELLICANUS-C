#ifndef PELLICANUS_H
#define PELLICANUS_H

/* I2C1 shared bus pins (MMC5983MA + MS5637) */
#define PELLICANUS_I2C_PORT  i2c1
#define PELLICANUS_PIN_SDA   26
#define PELLICANUS_PIN_SCL   27
#define PELLICANUS_I2C_SPEED 400000   /* 400 kHz: 1 kOhm pull-ups on the board */

/* IMU sampling rate */
#define PELLICANUS_IMU_HZ    100
/* GNSS update rate */
#define PELLICANUS_GNSS_HZ   4

/* IMU full-scale ranges. A vehicle's vibration and manoeuvres clip +-2 g and +-500 deg/s; the
 * widest ranges cost resolution the sensor's noise already hides. */
#define PELLICANUS_ACCEL_FS  IAM20680_ACCEL_FS_16G
#define PELLICANUS_GYRO_FS   IAM20680_GYRO_FS_2000DPS

#include "iam20680.h"
#include "mmc5983ma.h"
#include "ms5637.h"
#include "sl871_gnss.h"

#endif /* PELLICANUS_H */
