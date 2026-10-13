#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The binary stream over USB (docs/protocol.md).
 *
 * Every packet, little-endian:
 *
 *   0xA5 0x5A | type u8 | version u8 | length u16 | seq u32 | time_us u64 | payload | crc u16
 *
 * `length` is the payload's; `seq` counts the packets of that type; `time_us` is the RP2040's
 * time_us_64() for the measurement; `crc` is CRC-16/CCITT-FALSE over type..payload.
 *
 * A packet is sent whole or not at all: when the host is not connected or not reading fast enough
 * it is dropped and counted, and nothing ever waits for USB.
 */

#define PROTOCOL_SYNC0 0xA5
#define PROTOCOL_SYNC1 0x5A
#define PROTOCOL_VERSION 1
#define PROTOCOL_HEADER_BYTES 18
#define PROTOCOL_MAX_PAYLOAD 255

typedef enum {
    PROTOCOL_IMU = 0x01,       /* imu_payload_t */
    PROTOCOL_MAG = 0x02,       /* mag_payload_t */
    PROTOCOL_BARO = 0x03,      /* baro_payload_t */
    PROTOCOL_GNSS_NMEA = 0x04, /* one NMEA sentence, ASCII, without CR LF */
    PROTOCOL_STATUS = 0x05,    /* status_payload_t, once a second */
    PROTOCOL_TEXT = 0x06,      /* a message, ASCII */
} protocol_type_t;

/* In each sensor's own axes, right-handed, as its datasheet draws them. */
typedef struct __attribute__((packed)) {
    float accel[3]; /* m/s^2 */
    float gyro[3];  /* rad/s */
    float temperature; /* C, the IMU die */
} imu_payload_t;

typedef struct __attribute__((packed)) {
    float field[3]; /* gauss */
} mag_payload_t;

typedef struct __attribute__((packed)) {
    float pressure;    /* Pa */
    float temperature; /* C */
} baro_payload_t;

#define STATUS_SENSOR_IMU  0x01
#define STATUS_SENSOR_MAG  0x02
#define STATUS_SENSOR_BARO 0x04

typedef struct __attribute__((packed)) {
    char firmware[12];       /* version, NUL-padded */
    uint8_t sensors_ok;      /* STATUS_SENSOR_* that initialised */
    uint8_t imu_decimation;  /* sensor samples averaged into one output sample */
    uint16_t imu_rate_hz;    /* output rate */
    uint16_t imu_sensor_hz;  /* the sensor's own rate */
    uint16_t accel_range_g;
    uint16_t gyro_range_dps;
    uint32_t imu_samples;    /* output samples produced */
    uint32_t imu_dropped;    /* output samples lost: the first core did not take them in time */
    uint32_t imu_missed;     /* sensor samples not read before the next one */
    uint32_t mag_samples;
    uint32_t baro_samples;
    uint32_t gnss_sentences;
    uint32_t usb_dropped;    /* packets not sent: host not connected or not reading */
} status_payload_t;

/* Send one packet; false when it was dropped. */
bool protocol_send(protocol_type_t type, uint32_t seq, uint64_t time_us, const void *payload,
                   uint16_t length);

/* Packets dropped so far. */
uint32_t protocol_dropped(void);

uint16_t protocol_crc16(const uint8_t *data, size_t length);

#endif /* PROTOCOL_H */
