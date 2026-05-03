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

/* CTRL2 bits – continuous mode */
#define MMC5983MA_CTRL2_CMM_EN  (1 << 3)
#define MMC5983MA_CTRL2_ODR_100HZ 0x04     /* 100 Hz ODR */

/* STATUS bits */
#define MMC5983MA_STATUS_MEAS_M_DONE (1 << 0)

typedef struct {
    float mag_x, mag_y, mag_z;   /* Gauss */
    float temp;                   /* Celsius */
} mmc5983ma_data_t;

bool mmc5983ma_init(void);
bool mmc5983ma_read(mmc5983ma_data_t *out);

#endif /* MMC5983MA_H */
