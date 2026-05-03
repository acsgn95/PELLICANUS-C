#include "iam20680.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

/* Sensitivity divisors matching the Python driver (8g / 500dps defaults) */
static float accel_scale = 1.0f / 4096.0f;   /* 8g  → 4096 LSB/g  */
static float gyro_scale  = 1.0f / 65.5f;      /* 500dps → 65.5 LSB/dps */

static inline void cs_low(void)  { gpio_put(IAM20680_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(IAM20680_PIN_CS, 1); }

static void spi_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7F, val };
    cs_low();
    spi_write_blocking(IAM20680_SPI_PORT, buf, 2);
    cs_high();
}

static uint8_t spi_read_reg(uint8_t reg)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    cs_low();
    spi_write_blocking(IAM20680_SPI_PORT, &tx, 1);
    spi_read_blocking(IAM20680_SPI_PORT, 0x00, &rx, 1);
    cs_high();
    return rx;
}

static void spi_read_burst(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx = reg | 0x80;
    cs_low();
    spi_write_blocking(IAM20680_SPI_PORT, &tx, 1);
    spi_read_blocking(IAM20680_SPI_PORT, 0x00, buf, len);
    cs_high();
}

bool iam20680_init(iam20680_accel_fs_t accel_fs, iam20680_gyro_fs_t gyro_fs)
{
    spi_init(IAM20680_SPI_PORT, IAM20680_SPI_SPEED);
    /* Mode 3: CPOL=1, CPHA=1 */
    spi_set_format(IAM20680_SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(IAM20680_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(IAM20680_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(IAM20680_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(IAM20680_PIN_CS);
    gpio_set_dir(IAM20680_PIN_CS, GPIO_OUT);
    cs_high();
    sleep_ms(10);

    if (spi_read_reg(IAM20680_REG_WHO_AM_I) != IAM20680_WHO_AM_I_VAL)
        return false;

    /* Wake up, use best available clock */
    spi_write_reg(IAM20680_REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);

    spi_write_reg(IAM20680_REG_ACCEL_CONFIG,  (uint8_t)accel_fs);
    spi_write_reg(IAM20680_REG_GYRO_CONFIG,   (uint8_t)gyro_fs);
    /* DLPF ~92 Hz bandwidth */
    spi_write_reg(IAM20680_REG_CONFIG,        0x02);
    /* ODR = 1 kHz / (1+0) = 1 kHz; trimmed by the 100 Hz timer in main */
    spi_write_reg(IAM20680_REG_SMPLRT_DIV,    0x00);

    /* Update scale factors */
    switch (accel_fs) {
        case IAM20680_ACCEL_FS_2G:  accel_scale = 1.0f / 16384.0f; break;
        case IAM20680_ACCEL_FS_4G:  accel_scale = 1.0f / 8192.0f;  break;
        case IAM20680_ACCEL_FS_8G:  accel_scale = 1.0f / 4096.0f;  break;
        case IAM20680_ACCEL_FS_16G: accel_scale = 1.0f / 2048.0f;  break;
    }
    switch (gyro_fs) {
        case IAM20680_GYRO_FS_250DPS:  gyro_scale = 1.0f / 131.0f;  break;
        case IAM20680_GYRO_FS_500DPS:  gyro_scale = 1.0f / 65.5f;   break;
        case IAM20680_GYRO_FS_1000DPS: gyro_scale = 1.0f / 32.8f;   break;
        case IAM20680_GYRO_FS_2000DPS: gyro_scale = 1.0f / 16.4f;   break;
    }

    return true;
}

bool iam20680_read(iam20680_data_t *out)
{
    /* Burst-read 14 bytes: ACCEL(6) + TEMP(2) + GYRO(6) */
    uint8_t raw[14];
    spi_read_burst(IAM20680_REG_ACCEL_XOUT_H, raw, 14);

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t t  = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    /* NED frame: X→forward, Y→right, Z→down  (matches Python driver sign flips) */
    out->accel_x =  (float)ax * accel_scale * 9.80665f;
    out->accel_y =  (float)ay * accel_scale * 9.80665f;
    out->accel_z = -(float)az * accel_scale * 9.80665f;

    out->gyro_x =  (float)gx * gyro_scale;
    out->gyro_y =  (float)gy * gyro_scale;
    out->gyro_z = -(float)gz * gyro_scale;

    out->temp = (float)t / 340.0f + 36.53f;

    return true;
}
