#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pellicanus.h"
#include "imu_stream.h"
#include "protocol.h"

#define PIN_LED_SENSOR  0

/* ------------------------------------------------------------------ */
/*  Shared state updated by timers                                     */
/* ------------------------------------------------------------------ */
static volatile bool gnss_ready = false;

static mmc5983ma_data_t mag_data;
static ms5637_data_t    baro_data;
static sl871_data_t     gnss_data;
static absolute_time_t  next_mag_poll;

/* 4 Hz GNSS timer callback */
static bool gnss_timer_cb(repeating_timer_t *rt)
{
    (void)rt;
    gnss_ready = true;
    return true;
}

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
/*  main                                                                */
/* ------------------------------------------------------------------ */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);   /* wait for USB CDC enumeration */

    gpio_init(PIN_LED_SENSOR);
    gpio_set_dir(PIN_LED_SENSOR, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(PIN_LED_SENSOR, 0);
        sleep_ms(200);
        gpio_put(PIN_LED_SENSOR, 1);
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
    send_text("[OK] SL-871 GNSS initialised");

    gpio_put(PIN_LED_SENSOR, !(imu_ok && mag_ok && baro_ok));

    repeating_timer_t gnss_timer;
    /* Negative period → fire every |period| ms regardless of callback duration */
    add_repeating_timer_ms(-(1000 / PELLICANUS_GNSS_HZ), gnss_timer_cb, NULL, &gnss_timer);

    if (imu_ok)
        imu_stream_start();

    send_text("PELLICANUS %s ready: IMU %d Hz (%d Hz sensor, mean of %d), protocol %d",
              PELLICANUS_FIRMWARE_VERSION, PELLICANUS_IMU_HZ, PELLICANUS_IMU_SENSOR_HZ,
              PELLICANUS_IMU_DECIMATION, PROTOCOL_VERSION);

    absolute_time_t next_status = make_timeout_time_ms(1000);
    while (true) {
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

        if (gnss_ready) {
            gnss_ready = false;
            sl871_update(&gnss_data); /* sentences go out through send_nmea as they complete */
        }

        if (time_reached(next_status)) {
            next_status = make_timeout_time_ms(1000);
            send_status();
        }
    }
}
