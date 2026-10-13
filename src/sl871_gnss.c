#include "sl871_gnss.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static char nmea_buf[SL871_NMEA_BUF_LEN];
static int  nmea_pos = 0;

static sl871_data_t current;
static sl871_sentence_handler_t sentence_handler;

void sl871_set_sentence_handler(sl871_sentence_handler_t handler)
{
    sentence_handler = handler;
}

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

/* ------------------------------------------------------------------ */
/*  Receive: every byte goes into a ring buffer from the UART interrupt */
/* ------------------------------------------------------------------ */
/* The UART's FIFO holds 32 bytes, under 3 ms at 115200 baud: a loop that polls it loses bytes. */
#define RX_BUFFER 2048 /* a power of two */
static uint8_t rx[RX_BUFFER];
static volatile uint32_t rx_head, rx_tail;
static volatile uint32_t rx_overflow;
static int detected_baud;

static void on_uart_rx(void)
{
    while (uart_is_readable(SL871_UART_PORT)) {
        const uint8_t c = (uint8_t)uart_getc(SL871_UART_PORT);
        if (rx_head - rx_tail < RX_BUFFER)
            rx[rx_head++ % RX_BUFFER] = c;
        else
            rx_overflow++;
    }
}

static bool rx_pop(uint8_t *c)
{
    if (rx_tail == rx_head)
        return false;
    *c = rx[rx_tail++ % RX_BUFFER];
    return true;
}

static void set_baud(int baud)
{
    uart_set_baudrate(SL871_UART_PORT, (uint)baud);
    rx_tail = rx_head; /* what came at the old rate is noise */
    nmea_pos = 0;
}

/* Whether a sentence's checksum (XOR of the characters between $ and *) matches. */
static bool checksum_ok(const char *line, int length)
{
    uint8_t sum = 0;
    int i = 1;
    for (; i < length && line[i] != '*'; i++)
        sum ^= (uint8_t)line[i];
    if (i + 2 >= length) /* no '*' followed by two hex digits */
        return false;
    return (uint8_t)strtoul(&line[i + 1], NULL, 16) == sum;
}

/* Whether a sentence with a valid checksum arrives within `ms`. */
static bool hears_sentence(uint32_t ms)
{
    char line[SL871_NMEA_BUF_LEN];
    int n = 0;
    const absolute_time_t until = make_timeout_time_ms(ms);
    while (!time_reached(until)) {
        uint8_t c;
        if (!rx_pop(&c)) {
            sleep_us(200);
            continue;
        }
        if (c == '$')
            n = 0;
        if (c == '\n' || c == '\r') {
            if (n > 6 && line[0] == '$' && checksum_ok(line, n))
                return true;
            n = 0;
        } else if (n < SL871_NMEA_BUF_LEN - 1) {
            line[n++] = (char)c;
        }
    }
    return false;
}

static void send_command(const char *body)
{
    uint8_t sum = 0;
    char command[64];
    for (const char *p = body; *p; p++)
        sum ^= (uint8_t)*p;
    snprintf(command, sizeof(command), "$%s*%02X\r\n", body, sum);
    uart_puts(SL871_UART_PORT, command);
}

/* Four fixes a second; the receiver answers with $PMTK001,220,3 (forwarded like any sentence). */
static void set_fix_rate(void)
{
    char body[24];
    snprintf(body, sizeof(body), "PMTK220,%d", SL871_FIX_PERIOD_MS);
    send_command(body);
}

void sl871_init(void)
{
    /* Out of reset: a short pulse, then held high, and time for the receiver to start. */
    gpio_init(SL871_PIN_RESET_N);
    gpio_set_dir(SL871_PIN_RESET_N, GPIO_OUT);
    gpio_put(SL871_PIN_RESET_N, 0);
    sleep_ms(10);
    gpio_put(SL871_PIN_RESET_N, 1);
    sleep_ms(500);

    uart_init(SL871_UART_PORT, SL871_BAUD_FAST);
    gpio_set_function(SL871_PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(SL871_PIN_RX, GPIO_FUNC_UART);
    uart_set_format(SL871_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SL871_UART_PORT, true);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(SL871_UART_PORT, true, false);

    memset(&current, 0, sizeof(current));

    /* The receiver may already be at 115200 (set earlier, kept on its backup supply); otherwise it
       starts at 9600 and is moved up. A command without its checksum is ignored by the receiver,
       which is why the baud change used to fail and nothing was heard at all. */
    if (hears_sentence(1500)) {
        detected_baud = SL871_BAUD_FAST;
        set_fix_rate();
        return;
    }
    set_baud(SL871_BAUD_INIT);
    if (!hears_sentence(1500)) {
        detected_baud = 0; /* nothing at either rate */
        set_baud(SL871_BAUD_FAST);
        return;
    }
    send_command("PMTK251,115200");
    sleep_ms(100);
    set_baud(SL871_BAUD_FAST);
    if (hears_sentence(1500)) {
        detected_baud = SL871_BAUD_FAST;
        set_fix_rate();
        return;
    }
    set_baud(SL871_BAUD_INIT); /* it did not move: stay where it talks */
    detected_baud = SL871_BAUD_INIT;
}

int sl871_baud(void)
{
    return detected_baud;
}

uint32_t sl871_overflow(void)
{
    return rx_overflow;
}

bool sl871_update(sl871_data_t *out)
{
    bool got_sentence = false;
    uint8_t byte;

    while (rx_pop(&byte)) {
        const char c = (char)byte;
        if (c == '\n') {
            nmea_buf[nmea_pos] = '\0';
            if (nmea_pos > 0 && nmea_buf[0] == '$') {
                if (sentence_handler)
                    sentence_handler(nmea_buf, nmea_pos);
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
