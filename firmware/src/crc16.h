#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#define CRC16_INIT 0xFFFF

uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);   // CRC-16/CCITT-FALSE
uint16_t crc16(const uint8_t *data, uint16_t len);

#endif
