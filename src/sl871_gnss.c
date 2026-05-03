#include "sl871_gnss.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char nmea_buf[SL871_NMEA_BUF_LEN];
static int  nmea_pos = 0;

static sl871_data_t current;

/* Convert NMEA ddmm.mmmm to decimal degrees */
static double nmea_to_deg(const char *s, char dir)
{
    if (!s || s[0] == '\0') return 0.0;
    double raw = atof(s);
    int    deg = (int)(raw / 100);
    double min = raw - deg * 100.0;
    double dd  = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dd = -dd;
    return dd;
}

/* Parse $GNGGA sentence */
static void parse_gga(char *s)
{
    /* $GNGGA,hhmmss.ss,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,,*cs */
    char *fields[16];
    int  n = 0;
    char *p = strtok(s, ",");
    while (p && n < 16) { fields[n++] = p; p = strtok(NULL, ","); }
    if (n < 10) return;

    int fix = atoi(fields[6]);
    if (fix == 0) { current.valid = false; return; }

    current.latitude   = nmea_to_deg(fields[2], fields[3][0]);
    current.longitude  = nmea_to_deg(fields[4], fields[5][0]);
    current.fix_quality = (uint8_t)fix;
    current.satellites  = (uint8_t)atoi(fields[7]);
    current.altitude    = (float)atof(fields[9]);
    current.geoid_sep   = (n > 11) ? (float)atof(fields[11]) : 0.0f;
    current.valid       = true;
}

/* Parse $GNVTG sentence */
static void parse_vtg(char *s)
{
    /* $GNVTG,heading,T,,M,speed_knots,N,speed_kmh,K,mode*cs */
    char *fields[10];
    int  n = 0;
    char *p = strtok(s, ",");
    while (p && n < 10) { fields[n++] = p; p = strtok(NULL, ","); }
    if (n < 8) return;

    current.heading      = (float)atof(fields[1]);
    float speed_kmh      = (float)atof(fields[7]);
    current.ground_speed = speed_kmh / 3.6f;   /* km/h → m/s */
}

static void process_sentence(char *line)
{
    if (strncmp(line, "$GNGGA", 6) == 0)
        parse_gga(line);
    else if (strncmp(line, "$GNVTG", 6) == 0)
        parse_vtg(line);
}

void sl871_init(void)
{
    uart_init(SL871_UART_PORT, SL871_BAUD_INIT);
    gpio_set_function(SL871_PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(SL871_PIN_RX, GPIO_FUNC_UART);
    uart_set_format(SL871_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SL871_UART_PORT, true);

    sleep_ms(500);

    /* Cold-start then switch to 115200 baud */
    uart_puts(SL871_UART_PORT, "$PMTK101\r\n");
    sleep_ms(100);
    uart_puts(SL871_UART_PORT, "$PMTK258,1,2,115200,1,1\r\n");
    sleep_ms(100);

    uart_init(SL871_UART_PORT, SL871_BAUD_FAST);
    uart_set_format(SL871_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SL871_UART_PORT, true);

    memset(&current, 0, sizeof(current));
}

bool sl871_update(sl871_data_t *out)
{
    bool got_sentence = false;

    while (uart_is_readable(SL871_UART_PORT)) {
        char c = (char)uart_getc(SL871_UART_PORT);
        if (c == '\n') {
            nmea_buf[nmea_pos] = '\0';
            if (nmea_pos > 0 && nmea_buf[0] == '$') {
                process_sentence(nmea_buf);
                got_sentence = true;
            }
            nmea_pos = 0;
        } else if (c != '\r') {
            if (nmea_pos < SL871_NMEA_BUF_LEN - 1)
                nmea_buf[nmea_pos++] = c;
        }
    }

    if (out)
        *out = current;

    return got_sentence && current.valid;
}
