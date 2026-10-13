#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* I2C bus shared with MS5637 */
#define MMC5983MA_I2C_PORT   i2c1
#define MMC5983MA_I2C_ADDR   0x30

/* Register map */
#define MMC5983MA_REG_XOUT_0    0x00
#define MMC5983MA_REG_XOUT_1    0x01
#define MMC5983MA_REG_YOUT_0    0x02
#define MMC5983MA_REG_YOUT_1    0x03
#define MMC5983MA_REG_ZOUT_0    0x04
#define MMC5983MA_REG_ZOUT_1    0x05
#define MMC5983MA_REG_XYZOUT_2  0x06
#define MMC5983MA_REG_TOUT      0x07
#define MMC5983MA_REG_STATUS    0x08
#define MMC5983MA_REG_CTRL0     0x09
#define MMC5983MA_REG_CTRL1     0x0A
#define MMC5983MA_REG_CTRL2     0x0B
#define MMC5983MA_REG_CTRL3     0x0C
#define MMC5983MA_REG_PRODUCT_ID 0x2F

#define MMC5983MA_PRODUCT_ID_VAL 0x30

/* CTRL0 bits */
#define MMC5983MA_CTRL0_TM_M    (1 << 0)   /* Take measurement */
#define MMC5983MA_CTRL0_SET     (1 << 3)
#define MMC5983MA_CTRL0_RESET   (1 << 4)
#define MMC5983MA_CTRL0_AUTO_SR_EN (1 << 5) /* automatic SET/RESET */

/* CTRL1: bandwidth, i.e. measurement time; 100 Hz is 8 ms, enough for 100 Hz continuous mode */
#define MMC5983MA_CTRL1_BW_100HZ 0x00

/* CTRL2 bits – continuous mode. CM_FREQ codes: 1 Hz 1, 10 Hz 2, 20 Hz 3, 50 Hz 4, 100 Hz 5,
 * 200 Hz 6, 1000 Hz 7 (0x04, used before as "100 Hz", is 50 Hz). */
#define MMC5983MA_CTRL2_CMM_EN    (1 << 3)
#define MMC5983MA_CTRL2_CM_100HZ  0x05
#define MMC5983MA_CTRL2_PRD_SET_100 (0x3 << 4) /* a SET every 100 measurements */
#define MMC5983MA_CTRL2_EN_PRD_SET  (1 << 7)   /* periodic SET (needs AUTO_SR_EN) */

/* STATUS bits */
#define MMC5983MA_STATUS_MEAS_M_DONE (1 << 0)

typedef struct {
    float mag_x, mag_y, mag_z;   /* Gauss */
    float temp;                   /* Celsius; not measured in continuous mode (0) */
} mmc5983ma_data_t;

bool mmc5983ma_init(void);
/* Reads the output registers as they are. */
bool mmc5983ma_read(mmc5983ma_data_t *out);
/* Non-blocking: true when a new measurement was ready and is now in *out. */
bool mmc5983ma_poll(mmc5983ma_data_t *out);

#endif /* MMC5983MA_H */
