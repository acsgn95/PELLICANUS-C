#ifndef MS5637_H
#define MS5637_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#define MS5637_I2C_PORT   i2c1
#define MS5637_I2C_ADDR   0x76

/* Commands */
#define MS5637_CMD_RESET    0x1E
#define MS5637_CMD_PROM_C1  0xA2
#define MS5637_CMD_PROM_C2  0xA4
#define MS5637_CMD_PROM_C3  0xA6
#define MS5637_CMD_PROM_C4  0xA8
#define MS5637_CMD_PROM_C5  0xAA
#define MS5637_CMD_PROM_C6  0xAC
#define MS5637_CMD_READ_ADC 0x00
#define MS5637_CMD_CONV_D1  0x4A   /* Pressure OSR=8192 */
#define MS5637_CMD_CONV_D2  0x54   /* Temperature OSR=1024 */

typedef struct {
    float pressure;      /* mbar (hPa) */
    float temperature;   /* Celsius */
    float altitude;      /* meters above sea level (approx) */
} ms5637_data_t;

/* Worst-case conversion times: pressure at OSR 8192 18.08 ms (about 1.6 Pa RMS, against
 * about 5 Pa at OSR 1024), temperature at OSR 1024 2.28 ms (it changes slowly). */
#define MS5637_PRESSURE_CONVERSION_US    18500
#define MS5637_TEMPERATURE_CONVERSION_US 2500
/* A pressure and temperature pair every 25 ms: 40 Hz. */
#define MS5637_CYCLE_PERIOD_US 25000

bool ms5637_init(void);
/* Blocking: about 21 ms. */
bool ms5637_read(ms5637_data_t *out);
/* Non-blocking: call often; true when a new measurement is in *out. */
bool ms5637_poll(ms5637_data_t *out);

#endif /* MS5637_H */
