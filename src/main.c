#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pellicanus.h"

/* ------------------------------------------------------------------ */
/*  Shared state updated by timers                                     */
/* ------------------------------------------------------------------ */
static volatile bool imu_ready  = false;
static volatile bool gnss_ready = false;

static iam20680_data_t imu_data;
static mmc5983ma_data_t mag_data;
static ms5637_data_t    baro_data;
static sl871_data_t     gnss_data;

/* 100 Hz IMU timer callback */
static bool imu_timer_cb(repeating_timer_t *rt)
{
    (void)rt;
    imu_ready = true;
    return true;
}

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

    i2c1_init_bus();
    sl871_init();

    if (!iam20680_init(IAM20680_ACCEL_FS_8G, IAM20680_GYRO_FS_500DPS))
        printf("[ERROR] IAM-20680 not found\n");
    else
        printf("[OK] IAM-20680 initialised\n");

    if (!mmc5983ma_init())
        printf("[ERROR] MMC5983MA not found\n");
    else
        printf("[OK] MMC5983MA initialised\n");

    if (!ms5637_init())
        printf("[ERROR] MS5637 not found\n");
    else
        printf("[OK] MS5637 initialised\n");

    printf("[OK] SL-871 GNSS initialised\n");

    repeating_timer_t imu_timer, gnss_timer;
    /* Negative period → fire every |period| ms regardless of callback duration */
    add_repeating_timer_ms(-(1000 / PELLICANUS_IMU_HZ),  imu_timer_cb,  NULL, &imu_timer);
    add_repeating_timer_ms(-(1000 / PELLICANUS_GNSS_HZ), gnss_timer_cb, NULL, &gnss_timer);

    printf("PELLICANUS ready — streaming at %d Hz IMU / %d Hz GNSS\n",
           PELLICANUS_IMU_HZ, PELLICANUS_GNSS_HZ);

    while (true) {
        if (imu_ready) {
            imu_ready = false;
            iam20680_read(&imu_data);
            mmc5983ma_read(&mag_data);
            ms5637_read(&baro_data);

            printf("AX:%.4f AY:%.4f AZ:%.4f "
                   "GX:%.4f GY:%.4f GZ:%.4f "
                   "MX:%.4f MY:%.4f MZ:%.4f "
                   "P:%.2f T:%.2f ALT:%.2f",
                   imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                   imu_data.gyro_x,  imu_data.gyro_y,  imu_data.gyro_z,
                   mag_data.mag_x,   mag_data.mag_y,   mag_data.mag_z,
                   baro_data.pressure, baro_data.temperature, baro_data.altitude);

            if (gnss_data.valid) {
                printf(" LAT:%.7f LON:%.7f HGNSS:%.2f SPD:%.3f HDG:%.2f",
                       gnss_data.latitude, gnss_data.longitude,
                       gnss_data.altitude, gnss_data.ground_speed,
                       gnss_data.heading);
            }
            printf("\n");
        }

        if (gnss_ready) {
            gnss_ready = false;
            sl871_update(&gnss_data);
        }
    }
}
