#include "ms5637.h"
#include "pico/stdlib.h"
#include <math.h>

static uint16_t prom[6];   /* C1..C6 calibration coefficients */

static void send_cmd(uint8_t cmd)
{
    i2c_write_blocking(MS5637_I2C_PORT, MS5637_I2C_ADDR, &cmd, 1, false);
}

static uint16_t read_prom(uint8_t cmd)
{
    uint8_t buf[2];
    i2c_write_blocking(MS5637_I2C_PORT, MS5637_I2C_ADDR, &cmd, 1, true);
    i2c_read_blocking(MS5637_I2C_PORT, MS5637_I2C_ADDR, buf, 2, false);
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

static uint32_t read_adc(void)
{
    uint8_t cmd = MS5637_CMD_READ_ADC;
    uint8_t buf[3];
    i2c_write_blocking(MS5637_I2C_PORT, MS5637_I2C_ADDR, &cmd, 1, true);
    i2c_read_blocking(MS5637_I2C_PORT, MS5637_I2C_ADDR, buf, 3, false);
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

bool ms5637_init(void)
{
    /* Note: i2c1 hardware is initialised in main before calling this */
    send_cmd(MS5637_CMD_RESET);
    sleep_ms(10);

    prom[0] = read_prom(MS5637_CMD_PROM_C1);
    prom[1] = read_prom(MS5637_CMD_PROM_C2);
    prom[2] = read_prom(MS5637_CMD_PROM_C3);
    prom[3] = read_prom(MS5637_CMD_PROM_C4);
    prom[4] = read_prom(MS5637_CMD_PROM_C5);
    prom[5] = read_prom(MS5637_CMD_PROM_C6);

    return (prom[0] != 0 && prom[0] != 0xFFFF);
}

bool ms5637_read(ms5637_data_t *out)
{
    /* Convert D1 (pressure) */
    send_cmd(MS5637_CMD_CONV_D1);
    sleep_ms(3);   /* OSR=1024 max conversion time ~2.28 ms */
    uint32_t D1 = read_adc();

    /* Convert D2 (temperature) */
    send_cmd(MS5637_CMD_CONV_D2);
    sleep_ms(3);
    uint32_t D2 = read_adc();

    /* First-order compensation (MS5637 datasheet section 4.9.1) */
    int32_t dT   = (int32_t)D2 - ((int32_t)prom[4] << 8);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * prom[5]) >> 23);

    int64_t OFF  = ((int64_t)prom[1] << 17) + (((int64_t)prom[3] * dT) >> 6);
    int64_t SENS = ((int64_t)prom[0] << 16) + (((int64_t)prom[2] * dT) >> 7);

    /* Second-order low-temperature compensation */
    if (TEMP < 2000) {
        int32_t T2    = (int32_t)(((int64_t)dT * dT) >> 31);
        int64_t OFF2  = 61LL * (TEMP - 2000) * (TEMP - 2000) / 16;
        int64_t SENS2 = 29LL * (TEMP - 2000) * (TEMP - 2000) / 16;
        if (TEMP < -1500) {
            OFF2  += 17LL * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 9LL  * (TEMP + 1500) * (TEMP + 1500);
        }
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (int32_t)(((int64_t)D1 * SENS / 2097152LL - OFF) / 32768LL);

    out->temperature = (float)TEMP / 100.0f;
    out->pressure    = (float)P    / 100.0f;

    /* Hypsometric formula for altitude estimate */
    out->altitude = 44330.0f * (1.0f - powf(out->pressure / 1013.25f, 0.1903f));

    return true;
}
