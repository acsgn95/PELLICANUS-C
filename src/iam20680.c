#include "iam20680.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

/* Sensitivity divisors matching the Python driver (8g / 500dps defaults) */
static float accel_scale = 1.0f / 4096.0f;   /* 8g  → 4096 LSB/g  */
static float gyro_scale  = 1.0f / 65.5f;      /* 500dps → 65.5 LSB/dps */
static uint8_t who_am_i = 0;                 /* what WHO_AM_I read at init */

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

    /* The IAM-20680 family shares the register map; each part answers its own WHO_AM_I. */
    who_am_i = spi_read_reg(IAM20680_REG_WHO_AM_I);
    if (strcmp(iam20680_part(), "unknown") == 0)
        return false;

    /* Full device reset */
    spi_write_reg(IAM20680_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    /* Wake up, use best available clock */
    spi_write_reg(IAM20680_REG_PWR_MGMT_1, 0x01);
    sleep_ms(50);

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

void iam20680_read_raw(iam20680_raw_t *raw)
{
    /* Burst-read 14 bytes: ACCEL(6) + TEMP(2) + GYRO(6) */
    uint8_t buf[14];
    spi_read_burst(IAM20680_REG_ACCEL_XOUT_H, buf, 14);
    for (int i = 0; i < 7; i++)
        raw->v[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);
}

void iam20680_convert(const int32_t sum[7], int n, iam20680_data_t *out)
{
    const float k = 1.0f / (float)n;

    /* NED frame: X→forward, Y→right, Z→down  (matches Python driver sign flips) */
    out->accel_x =  (float)sum[0] * k * accel_scale * 9.80665f;
    out->accel_y =  (float)sum[1] * k * accel_scale * 9.80665f;
    out->accel_z = -(float)sum[2] * k * accel_scale * 9.80665f;

    out->gyro_x =  (float)sum[4] * k * gyro_scale;
    out->gyro_y =  (float)sum[5] * k * gyro_scale;
    out->gyro_z = -(float)sum[6] * k * gyro_scale;

    out->temp = (float)sum[3] * k / 340.0f + 36.53f;
}

bool iam20680_read(iam20680_data_t *out)
{
    iam20680_raw_t raw;
    int32_t sum[7];

    iam20680_read_raw(&raw);
    for (int i = 0; i < 7; i++)
        sum[i] = raw.v[i];
    iam20680_convert(sum, 1, out);
    return true;
}

void iam20680_start_stream(uint8_t gyro_dlpf_cfg, uint8_t accel_dlpf_cfg)
{
    spi_write_reg(IAM20680_REG_CONFIG, gyro_dlpf_cfg & 0x07);        /* gyro and temperature DLPF */
    spi_write_reg(IAM20680_REG_ACCEL_CONFIG2, accel_dlpf_cfg & 0x07); /* accelerometer DLPF */
    spi_write_reg(IAM20680_REG_SMPLRT_DIV, 0x00);                     /* 1 kHz / (1 + 0) */
    /* INT active high, push-pull, held until cleared, cleared by any register read. */
    spi_write_reg(IAM20680_REG_INT_PIN_CFG, 0x30);
    spi_write_reg(IAM20680_REG_INT_ENABLE, 0x01);                     /* data ready */
}

uint8_t iam20680_who_am_i(void)
{
    return who_am_i;
}

const char *iam20680_part(void)
{
    switch (who_am_i) {
        case IAM20680_WHO_AM_I_VAL:   return "IAM-20680";
        case IAM20680HP_WHO_AM_I_VAL: return "IAM-20680HP";
        case IAM20680HT_WHO_AM_I_VAL: return "IAM-20680HT";
        default:                      return "unknown";
    }
}
