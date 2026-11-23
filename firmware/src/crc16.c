#include "crc16.h"

uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16(const uint8_t *data, uint16_t len)
{
    return crc16_update(CRC16_INIT, data, len);
}
