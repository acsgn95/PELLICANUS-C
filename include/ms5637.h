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
#define MS5637_CMD_CONV_D1  0x44   /* Pressure OSR=1024 */
#define MS5637_CMD_CONV_D2  0x54   /* Temperature OSR=1024 */

typedef struct {
    float pressure;      /* mbar (hPa) */
    float temperature;   /* Celsius */
    float altitude;      /* meters above sea level (approx) */
} ms5637_data_t;

bool ms5637_init(void);
bool ms5637_read(ms5637_data_t *out);

#endif /* MS5637_H */
