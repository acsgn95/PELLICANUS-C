#include "mmc5983ma.h"
#include "pico/stdlib.h"

/* Sensitivity: 16384 counts/Gauss for the 18-bit output */
#define MMC5983MA_SENSITIVITY  16384.0f
/* 18-bit zero-field offset (2^17) */
#define MMC5983MA_ZERO_OFFSET  131072.0f

static void i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(MMC5983MA_I2C_PORT, MMC5983MA_I2C_ADDR, buf, 2, false);
}

static uint8_t i2c_read_reg(uint8_t reg)
{
    uint8_t val;
    i2c_write_blocking(MMC5983MA_I2C_PORT, MMC5983MA_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(MMC5983MA_I2C_PORT, MMC5983MA_I2C_ADDR, &val, 1, false);
    return val;
}

static void i2c_read_burst(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_write_blocking(MMC5983MA_I2C_PORT, MMC5983MA_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(MMC5983MA_I2C_PORT, MMC5983MA_I2C_ADDR, buf, len, false);
}

bool mmc5983ma_init(void)
{
    /* Note: i2c1 hardware is initialised in main before calling this */
    if (i2c_read_reg(MMC5983MA_REG_PRODUCT_ID) != MMC5983MA_PRODUCT_ID_VAL)
        return false;

    /* SET pulse to initialize the bridge */
    i2c_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET);
    sleep_ms(1);

    /* Continuous measurement at 100 Hz (8 ms measurements), with automatic SET/RESET and a SET
       pulse every 100 measurements (about once a second): without it the bridge's offset moves
       with temperature and after a strong field. */
    i2c_write_reg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_BW_100HZ);
    i2c_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_AUTO_SR_EN);
    i2c_write_reg(MMC5983MA_REG_CTRL2, MMC5983MA_CTRL2_CMM_EN | MMC5983MA_CTRL2_CM_100HZ |
                                           MMC5983MA_CTRL2_EN_PRD_SET | MMC5983MA_CTRL2_PRD_SET_100);

    return true;
}

bool mmc5983ma_poll(mmc5983ma_data_t *out)
{
    /* Continuous mode measures by itself; read only when a measurement is ready, never wait. */
    if (!(i2c_read_reg(MMC5983MA_REG_STATUS) & MMC5983MA_STATUS_MEAS_M_DONE))
        return false;
    return mmc5983ma_read(out);
}

bool mmc5983ma_read(mmc5983ma_data_t *out)
{

    /* Read 7 bytes: XOUT0,XOUT1,YOUT0,YOUT1,ZOUT0,ZOUT1,XYZOUT2 */
    uint8_t raw[7];
    i2c_read_burst(MMC5983MA_REG_XOUT_0, raw, 7);

    uint32_t x18 = ((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) | ((raw[6] >> 6) & 0x03);
    uint32_t y18 = ((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) | ((raw[6] >> 4) & 0x03);
    uint32_t z18 = ((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) | ((raw[6] >> 2) & 0x03);

    out->mag_x = ((float)x18 - MMC5983MA_ZERO_OFFSET) / MMC5983MA_SENSITIVITY;
    out->mag_y = ((float)y18 - MMC5983MA_ZERO_OFFSET) / MMC5983MA_SENSITIVITY;
    out->mag_z = ((float)z18 - MMC5983MA_ZERO_OFFSET) / MMC5983MA_SENSITIVITY;

    /* The temperature register is only refreshed by a temperature measurement, which continuous
       mode does not make: reading it gave a stale value and cost an I2C transfer per sample. */
    out->temp = 0.0f;

    return true;
}
