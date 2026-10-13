#ifndef SL871_GNSS_H
#define SL871_GNSS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

#define SL871_UART_PORT     uart0
#define SL871_PIN_TX        28
#define SL871_PIN_RX        29
/* GNSS_RESET_N on the schematic: active low. The RP2040's pins come up as inputs with their
 * pull-downs on, which held the receiver in reset while nothing drove it. */
#define SL871_PIN_RESET_N   7
#define SL871_BAUD_INIT     9600
#define SL871_BAUD_FAST     115200
#define SL871_NMEA_BUF_LEN  256
/* Fix period set at start (PMTK220): 250 ms, 4 Hz. Only at 115200 baud: at 9600 the sentences of
 * four fixes a second do not fit in the line. */
#define SL871_FIX_PERIOD_MS 250

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

/* Called with every complete NMEA sentence (without CR LF) as it is received. */
typedef void (*sl871_sentence_handler_t)(const char *sentence, int length);

/* Sets up the UART with its receive interrupt and finds the receiver's baud rate, moving it to
 * 115200 when it talks at 9600: up to about 4.5 s. */
void sl871_init(void);
void sl871_set_sentence_handler(sl871_sentence_handler_t handler);
/* The baud rate the receiver was found at (115200 or 9600), 0 when it was not heard. */
int sl871_baud(void);
/* Bytes lost because the receive buffer was full (the main loop did not read in time). */
uint32_t sl871_overflow(void);
/* Call often (every main loop pass): parses what was received and hands each sentence to the
 * handler; fills *out with the latest fix, true when a sentence brought a valid one. */
bool sl871_update(sl871_data_t *out);

#endif /* SL871_GNSS_H */
