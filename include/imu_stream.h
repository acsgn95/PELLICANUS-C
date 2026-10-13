#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#include "iam20680.h"

/*
 * The IMU, sampled on core 1.
 *
 * The sensor produces a sample every 1 ms and raises INT as each one is ready. Core 1 does nothing
 * but wait for INT, take the time, read the sample and average a group of
 * PELLICANUS_IMU_DECIMATION samples into one output sample, which it hands to core 0 through a
 * queue. Nothing core 0 does (USB, the other sensors) can delay or drop an IMU sample; a sample
 * core 0 does not take in time is counted as dropped, and a 1 kHz sample core 1 itself misses is
 * counted as missed.
 */

typedef struct {
    /* Output sample number, consecutive from 0: a gap is a sample dropped. */
    uint32_t seq;
    /* time_us_64() at the sensor's data-ready, averaged over the samples in the group. The
     * sensor's low-pass filters delay the motion by a few milliseconds more (see README). */
    uint64_t time_us;
    iam20680_data_t data;
} imu_stream_sample_t;

/* Configure the sensor for streaming and start core 1. iam20680_init() must have succeeded. */
void imu_stream_start(void);

/* The next output sample, if there is one. Core 0 only. */
bool imu_stream_pop(imu_stream_sample_t *sample);

/* Output samples lost because the queue was full (core 0 too slow). */
uint32_t imu_stream_dropped(void);

/* 1 kHz samples core 1 did not read before the next one replaced them. */
uint32_t imu_stream_missed(void);

/* Output samples produced so far. */
uint32_t imu_stream_produced(void);

#endif /* IMU_STREAM_H */
