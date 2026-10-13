#include <inttypes.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pellicanus.h"
#include "imu_stream.h"

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

    bool imu_ok  = iam20680_init(PELLICANUS_ACCEL_FS, PELLICANUS_GYRO_FS);
    bool mag_ok  = mmc5983ma_init();
    bool baro_ok = ms5637_init();

    if (imu_ok)
        printf("[OK] %s initialised (WHO_AM_I 0x%02X)\n", iam20680_part(), iam20680_who_am_i());
    else
        printf("[ERROR] IAM-20680 family IMU not found (WHO_AM_I read 0x%02X)\n",
               iam20680_who_am_i());
    printf(mag_ok  ? "[OK] MMC5983MA initialised\n"  : "[ERROR] MMC5983MA not found\n");
    printf(baro_ok ? "[OK] MS5637 initialised\n"     : "[ERROR] MS5637 not found\n");
    printf("[OK] SL-871 GNSS initialised\n");

    gpio_put(PIN_LED_SENSOR, !(imu_ok && mag_ok && baro_ok));

    repeating_timer_t gnss_timer;
    /* Negative period → fire every |period| ms regardless of callback duration */
    add_repeating_timer_ms(-(1000 / PELLICANUS_GNSS_HZ), gnss_timer_cb, NULL, &gnss_timer);

    if (imu_ok)
        imu_stream_start();

    /* One line per measurement, each with the RP2040's time in microseconds:
     *   I seq t_us ax ay az gx gy gz temp   IMU, PELLICANUS_IMU_HZ (m/s^2, deg/s, C)
     *   M t_us mx my mz                     magnetometer, when it has a new measurement (G)
     *   B t_us pressure temperature         barometer, when it has a new measurement (hPa, C)
     *   G t_us lat lon height speed heading GNSS, when a sentence brings a valid fix
     *   S t_us imu_seq imu_dropped imu_missed   once a second */
    printf("PELLICANUS ready: IMU %d Hz (%d Hz sensor, mean of %d), GNSS %d Hz\n",
           PELLICANUS_IMU_HZ, PELLICANUS_IMU_SENSOR_HZ, PELLICANUS_IMU_DECIMATION,
           PELLICANUS_GNSS_HZ);

    absolute_time_t next_status = make_timeout_time_ms(1000);
    uint32_t last_seq = 0;
    while (true) {
        imu_stream_sample_t s;
        while (imu_stream_pop(&s)) {
            last_seq = s.seq;
            printf("I %" PRIu32 " %llu %.4f %.4f %.4f %.4f %.4f %.4f %.2f\n", s.seq,
                   (unsigned long long)s.time_us, s.data.accel_x, s.data.accel_y, s.data.accel_z, s.data.gyro_x,
                   s.data.gyro_y, s.data.gyro_z, s.data.temp);
        }

        /* Never wait on the slow sensors: each one is read when it has something new. The
           magnetometer measures at 100 Hz; its status is looked at every 2 ms, not on every pass,
           to leave the I2C bus free. */
        if (time_reached(next_mag_poll)) {
            next_mag_poll = make_timeout_time_us(2000);
            if (mmc5983ma_poll(&mag_data))
                printf("M %llu %.5f %.5f %.5f\n", (unsigned long long)time_us_64(), mag_data.mag_x,
                       mag_data.mag_y, mag_data.mag_z);
        }
        if (ms5637_poll(&baro_data))
            printf("B %llu %.3f %.2f\n", (unsigned long long)time_us_64(), baro_data.pressure,
                   baro_data.temperature);

        if (gnss_ready) {
            gnss_ready = false;
            if (sl871_update(&gnss_data))
                printf("G %llu %.7f %.7f %.2f %.3f %.2f\n", (unsigned long long)time_us_64(),
                       gnss_data.latitude, gnss_data.longitude, gnss_data.altitude,
                       gnss_data.ground_speed, gnss_data.heading);
        }

        if (time_reached(next_status)) {
            next_status = make_timeout_time_ms(1000);
            printf("S %llu %" PRIu32 " %" PRIu32 " %" PRIu32 "\n", (unsigned long long)time_us_64(), last_seq,
                   imu_stream_dropped(), imu_stream_missed());
        }
    }
}
