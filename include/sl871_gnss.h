#ifndef SL871_GNSS_H
#define SL871_GNSS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

#define SL871_UART_PORT     uart0
#define SL871_PIN_TX        28
#define SL871_PIN_RX        29
#define SL871_BAUD_INIT     9600
#define SL871_BAUD_FAST     115200
#define SL871_NMEA_BUF_LEN  256

typedef struct {
    double  latitude;        /* degrees, + = North */
    double  longitude;       /* degrees, + = East */
    float   altitude;        /* meters MSL */
    float   geoid_sep;       /* meters */
    float   ground_speed;    /* m/s */
    float   heading;         /* degrees true north */
    uint8_t fix_quality;     /* 0=invalid,1=GPS,2=DGPS */
    uint8_t satellites;
    bool    valid;
} sl871_data_t;

void sl871_init(void);
/* Call from a 4 Hz timer or polling loop; fills *out when a valid fix arrives */
bool sl871_update(sl871_data_t *out);

#endif /* SL871_GNSS_H */
