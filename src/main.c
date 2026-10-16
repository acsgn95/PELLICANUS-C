#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "pellicanus.h"
#include "imu_stream.h"
#include "protocol.h"


/* ------------------------------------------------------------------ */
/*  Shared state updated by timers                                     */
/* ------------------------------------------------------------------ */

static mmc5983ma_data_t mag_data;
static ms5637_data_t    baro_data;
static sl871_data_t     gnss_data;
static absolute_time_t  next_mag_poll;

/* ------------------------------------------------------------------ */
/*  Initialise shared I2C1 bus                                         */
/* ------------------------------------------------------------------ */
static void i2c1_init_bus(void)
{
    i2c_init(PELLICANUS_I2C_PORT, PELLICANUS_I2C_SPEED);
    gpio_set_function(PELLICANUS_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PELLICANUS_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PELLICANUS_PIN_SDA);
    gpio_pull_up(PELLICANUS_PIN_SCL);
}

/* ------------------------------------------------------------------ */
/*  Output: the binary protocol (protocol.h, docs/protocol.md)          */
/* ------------------------------------------------------------------ */
static uint32_t text_seq, mag_seq, baro_seq, gnss_seq, status_seq;
static uint8_t sensors_ok;

/* ------------------------------------------------------------------ */
/*  The health LED (D2), driven from the main loop without waiting      */
/* ------------------------------------------------------------------ */
typedef enum {
    LED_BOOT,     /* solid: starting the sensors */
    LED_OK,       /* one short flash a second: healthy, host reading */
    LED_NO_HOST,  /* two short flashes a second: healthy, nobody reading the stream */
    LED_FAULT,    /* five flashes a second: a sensor missing, samples lost, GNSS not heard */
} led_state_t;

static led_state_t led_state = LED_BOOT;

static void led_update(void)
{
    const uint32_t ms = to_ms_since_boot(get_absolute_time());
    const uint32_t phase = ms % 1000;
    bool on = false;
    switch (led_state) {
        case LED_BOOT:    on = true; break;
        case LED_OK:      on = phase < 80; break;
        case LED_NO_HOST: on = phase < 80 || (phase >= 200 && phase < 280); break;
        case LED_FAULT:   on = (ms % 200) < 100; break;
    }
    gpio_put(PELLICANUS_PIN_LED_HEALTH, on ? PELLICANUS_LED_ON : !PELLICANUS_LED_ON);
}

/* A message for the host, as a TEXT packet (dropped like any packet when nobody reads). */
static void send_text(const char *format, ...)
{
    char text[PROTOCOL_MAX_PAYLOAD];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (n < 0)
        return;
    if (n >= (int)sizeof(text))
        n = (int)sizeof(text) - 1;
    protocol_send(PROTOCOL_TEXT, text_seq++, time_us_64(), text, (uint16_t)n);
}

static void send_nmea(const char *sentence, int length)
{
    if (length > PROTOCOL_MAX_PAYLOAD)
        length = PROTOCOL_MAX_PAYLOAD;
    protocol_send(PROTOCOL_GNSS_NMEA, gnss_seq++, time_us_64(), sentence, (uint16_t)length);
}

static void send_status(void)
{
    status_payload_t st;
    memset(&st, 0, sizeof(st));
    strncpy(st.firmware, PELLICANUS_FIRMWARE_VERSION, sizeof(st.firmware));
    st.sensors_ok = sensors_ok;
    st.imu_decimation = PELLICANUS_IMU_DECIMATION;
    st.imu_rate_hz = PELLICANUS_IMU_HZ;
    st.imu_sensor_hz = PELLICANUS_IMU_SENSOR_HZ;
    st.accel_range_g = PELLICANUS_ACCEL_RANGE_G;
    st.gyro_range_dps = PELLICANUS_GYRO_RANGE_DPS;
    st.imu_samples = imu_stream_produced();
    st.imu_dropped = imu_stream_dropped();
    st.imu_missed = imu_stream_missed();
    st.mag_samples = mag_seq;
    st.baro_samples = baro_seq;
    st.gnss_sentences = gnss_seq;
    st.usb_dropped = protocol_dropped();
    protocol_send(PROTOCOL_STATUS, status_seq++, time_us_64(), &st, sizeof(st));
}

/* ------------------------------------------------------------------ */
/*  The watchdog                                                        */
/* ------------------------------------------------------------------ */
/* The board restarts itself when it stops working, rather than staying connected and silent:
   USB is serviced by an interrupt, so a main loop stuck in a driver (an I2C transfer waiting on
   a held bus has no timeout) still enumerates and opens, and sends nothing, until someone
   unplugs it. The main loop feeds the watchdog on every pass, and stops feeding it when core 1
   has produced no IMU sample for a while; either way the board restarts and enumerates again,
   and a host that looks for the board again finds it. It also stops feeding it when a host has
   the port open and no packet has gone out for two seconds: the board was seen running and
   connected while nothing reached the host until it was unplugged. */
#define WATCHDOG_START_MS 8000        /* during start-up: the USB wait and the sensors' init */
#define WATCHDOG_RUN_MS 1000          /* the main loop passes every few hundred microseconds */
#define WATCHDOG_IMU_STALL_US 1000000 /* 250 samples a second: a second without one is a stall */
/* A host has the port open, yet nothing has gone out for this long: the USB side is stuck. */
#define WATCHDOG_USB_STALL_US 2000000

/* ------------------------------------------------------------------ */
/*  main                                                                */
/* ------------------------------------------------------------------ */
int main(void)
{
    stdio_init_all();
    const bool watchdog_restart = watchdog_caused_reboot();
    watchdog_enable(WATCHDOG_START_MS, true);
    sleep_ms(2000);   /* wait for USB CDC enumeration */

    gpio_init(PELLICANUS_PIN_LED_HEALTH);
    gpio_set_dir(PELLICANUS_PIN_LED_HEALTH, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(PELLICANUS_PIN_LED_HEALTH, !PELLICANUS_LED_ON);
        sleep_ms(200);
        gpio_put(PELLICANUS_PIN_LED_HEALTH, PELLICANUS_LED_ON);
        sleep_ms(200);
    }

    i2c1_init_bus();
    sl871_init();
    sl871_set_sentence_handler(send_nmea);

    bool imu_ok  = iam20680_init(PELLICANUS_ACCEL_FS, PELLICANUS_GYRO_FS);
    bool mag_ok  = mmc5983ma_init();
    bool baro_ok = ms5637_init();
    sensors_ok = (imu_ok ? STATUS_SENSOR_IMU : 0) | (mag_ok ? STATUS_SENSOR_MAG : 0) |
                 (baro_ok ? STATUS_SENSOR_BARO : 0);

    if (imu_ok)
        send_text("[OK] %s initialised (WHO_AM_I 0x%02X)", iam20680_part(), iam20680_who_am_i());
    else
        send_text("[ERROR] IAM-20680 family IMU not found (WHO_AM_I read 0x%02X)",
                  iam20680_who_am_i());
    send_text(mag_ok  ? "[OK] MMC5983MA initialised"  : "[ERROR] MMC5983MA not found");
    send_text(baro_ok ? "[OK] MS5637 initialised"     : "[ERROR] MS5637 not found");
    if (sl871_baud() > 0)
        send_text("[OK] SL-871 GNSS heard at %d baud", sl871_baud());
    else
        send_text("[ERROR] SL-871 GNSS not heard at 115200 or 9600 baud");

    /* Solid until the loop starts: the LED's state is set once a second from the board's health. */
    led_update();

    if (watchdog_restart)
        send_text("[WARN] restarted by the watchdog: the main loop or the IMU stream had stopped");

    if (imu_ok)
        imu_stream_start();

    send_text("PELLICANUS %s ready: IMU %d Hz (%d Hz sensor, mean of %d), protocol %d",
              PELLICANUS_FIRMWARE_VERSION, PELLICANUS_IMU_HZ, PELLICANUS_IMU_SENSOR_HZ,
              PELLICANUS_IMU_DECIMATION, PROTOCOL_VERSION);

    absolute_time_t next_status = make_timeout_time_ms(1000);
    uint32_t lost_before = 0, usb_dropped_before = 0;
    const bool sensors_all = (sensors_ok == (STATUS_SENSOR_IMU | STATUS_SENSOR_MAG | STATUS_SENSOR_BARO));
    uint32_t gnss_overflow = 0;
    uint32_t imu_produced = imu_stream_produced();
    uint64_t imu_last_us = time_us_64();
    watchdog_enable(WATCHDOG_RUN_MS, true);
    while (true) {
        const uint64_t now_us = time_us_64();
        if (imu_stream_produced() != imu_produced) {
            imu_produced = imu_stream_produced();
            imu_last_us = now_us;
        }
        const bool imu_running = !imu_ok || now_us - imu_last_us < WATCHDOG_IMU_STALL_US;
        const uint64_t last_sent_us = protocol_last_sent_us();
        const bool usb_running = !stdio_usb_connected() || last_sent_us == 0 ||
                                 now_us - last_sent_us < WATCHDOG_USB_STALL_US;
        if (imu_running && usb_running)
            watchdog_update();

        imu_stream_sample_t s;
        while (imu_stream_pop(&s)) {
            const imu_payload_t p = {
                .accel = {s.data.accel_x, s.data.accel_y, s.data.accel_z},
                .gyro = {s.data.gyro_x, s.data.gyro_y, s.data.gyro_z},
                .temperature = s.data.temp,
            };
            protocol_send(PROTOCOL_IMU, s.seq, s.time_us, &p, sizeof(p));
        }

        /* Never wait on the slow sensors: each one is read when it has something new. The
           magnetometer's status is looked at every 2 ms, not on every pass, to leave the I2C bus
           free. */
        if (time_reached(next_mag_poll)) {
            next_mag_poll = make_timeout_time_us(2000);
            if (mmc5983ma_poll(&mag_data)) {
                const mag_payload_t p = {.field = {mag_data.mag_x, mag_data.mag_y, mag_data.mag_z}};
                protocol_send(PROTOCOL_MAG, mag_seq++, time_us_64(), &p, sizeof(p));
            }
        }
        if (ms5637_poll(&baro_data)) {
            const baro_payload_t p = {.pressure = baro_data.pressure * 100.0f,
                                      .temperature = baro_data.temperature};
            protocol_send(PROTOCOL_BARO, baro_seq++, time_us_64(), &p, sizeof(p));
        }

        /* Received by interrupt; parsed here. Each sentence goes out through send_nmea. */
        sl871_update(&gnss_data);

        if (time_reached(next_status)) {
            next_status = make_timeout_time_ms(1000);
            send_status();
            bool fault = !sensors_all || sl871_baud() == 0;
            if (sl871_overflow() != gnss_overflow) {
                gnss_overflow = sl871_overflow();
                fault = true;
                send_text("[WARN] GNSS receive buffer overflowed: %lu bytes lost so far",
                          (unsigned long)gnss_overflow);
            }
            const uint32_t lost = imu_stream_dropped() + imu_stream_missed();
            fault = fault || lost != lost_before;
            lost_before = lost;
            /* Nobody reading: not connected, or packets dropped for want of room this second. */
            const bool no_host = !stdio_usb_connected() || protocol_dropped() != usb_dropped_before;
            usb_dropped_before = protocol_dropped();
            led_state = fault ? LED_FAULT : no_host ? LED_NO_HOST : LED_OK;
        }
        led_update();
    }
}
