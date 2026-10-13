#include "protocol.h"

#include <string.h>

#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "tusb.h"

static uint32_t dropped;

uint16_t protocol_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF; /* CRC-16/CCITT-FALSE: polynomial 0x1021, no reflection */
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

bool protocol_send(protocol_type_t type, uint32_t seq, uint64_t time_us, const void *payload,
                   uint16_t length)
{
    uint8_t packet[PROTOCOL_HEADER_BYTES + PROTOCOL_MAX_PAYLOAD + 2];

    if (length > PROTOCOL_MAX_PAYLOAD) {
        dropped++;
        return false;
    }
    packet[0] = PROTOCOL_SYNC0;
    packet[1] = PROTOCOL_SYNC1;
    packet[2] = (uint8_t)type;
    packet[3] = PROTOCOL_VERSION;
    memcpy(&packet[4], &length, 2); /* the RP2040 is little-endian */
    memcpy(&packet[6], &seq, 4);
    memcpy(&packet[10], &time_us, 8);
    memcpy(&packet[PROTOCOL_HEADER_BYTES], payload, length);
    const size_t body = PROTOCOL_HEADER_BYTES + length;
    const uint16_t crc = protocol_crc16(&packet[2], body - 2);
    memcpy(&packet[body], &crc, 2);
    const size_t total = body + 2;

    /* Whole or not at all, and never wait: the USB stack's own write only waits when its buffer
       is full, so a packet that does not fit now is dropped here instead. */
    if (!stdio_usb_connected() || tud_cdc_write_available() < total) {
        dropped++;
        return false;
    }
    stdio_put_string((const char *)packet, (int)total, false, false);
    return true;
}

uint32_t protocol_dropped(void)
{
    return dropped;
}
