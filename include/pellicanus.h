#ifndef PELLICANUS_H
#define PELLICANUS_H

/* I2C1 shared bus pins (MMC5983MA + MS5637) */
#define PELLICANUS_I2C_PORT  i2c1
#define PELLICANUS_PIN_SDA   26
#define PELLICANUS_PIN_SCL   27
#define PELLICANUS_I2C_SPEED 100000   /* 100 kHz */

/* IMU sampling rate */
#define PELLICANUS_IMU_HZ    100
/* GNSS update rate */
#define PELLICANUS_GNSS_HZ   4

#include "iam20680.h"
#include "mmc5983ma.h"
#include "ms5637.h"
#include "sl871_gnss.h"

#endif /* PELLICANUS_H */
