#include "imu_stream.h"

#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "pellicanus.h"

/* About 256 ms of output at 250 Hz: core 0 may stall that long (USB) without a sample lost. */
#define IMU_STREAM_QUEUE_LENGTH 64

static queue_t queue;
static volatile uint32_t dropped;
static volatile uint32_t missed;

static void core1_main(void)
{
    int32_t sum[7] = {0};
    uint64_t time_sum = 0;
    uint64_t last = 0;
    uint32_t seq = 0;
    int n = 0;

    for (;;) {
        /* INT is held until the sample is read: a sample is never missed for being short. */
        while (!gpio_get(IAM20680_PIN_INT))
            tight_loop_contents();
        const uint64_t now = time_us_64();

        iam20680_raw_t raw;
        iam20680_read_raw(&raw); /* also clears INT */

        /* A gap of more than 1.5 periods is a sample the next one replaced. */
        if (last != 0 && now - last > 1500u)
            missed += (uint32_t)((now - last + 500u) / 1000u) - 1u;
        last = now;

        for (int i = 0; i < 7; i++)
            sum[i] += raw.v[i];
        time_sum += now;
        if (++n < PELLICANUS_IMU_DECIMATION)
            continue;

        imu_stream_sample_t sample;
        sample.seq = seq++;
        sample.time_us = time_sum / (uint64_t)n;
        iam20680_convert(sum, n, &sample.data);
        if (!queue_try_add(&queue, &sample))
            dropped++;

        for (int i = 0; i < 7; i++)
            sum[i] = 0;
        time_sum = 0;
        n = 0;
    }
}

void imu_stream_start(void)
{
    queue_init(&queue, sizeof(imu_stream_sample_t), IMU_STREAM_QUEUE_LENGTH);

    gpio_init(IAM20680_PIN_INT);
    gpio_set_dir(IAM20680_PIN_INT, GPIO_IN);
    gpio_pull_down(IAM20680_PIN_INT);

    iam20680_start_stream(PELLICANUS_IMU_GYRO_DLPF, PELLICANUS_IMU_ACCEL_DLPF);
    /* From here on core 1 owns the IMU's SPI bus. */
    multicore_launch_core1(core1_main);
}

bool imu_stream_pop(imu_stream_sample_t *sample)
{
    return queue_try_remove(&queue, sample);
}

uint32_t imu_stream_dropped(void)
{
    return dropped;
}

uint32_t imu_stream_missed(void)
{
    return missed;
}
