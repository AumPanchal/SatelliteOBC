#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#include <stdint.h>

// 16 byte record stored in flash
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t time_ms;
    uint16_t code;
    uint8_t  state;
    uint8_t  flags;
    uint16_t detail;
    uint16_t crc;
} log_rec_t;

int      log_init(void);                   // find the next free slot
uint16_t log_count(void);
int      log_write(uint16_t code, uint8_t state, uint8_t flags, uint16_t detail);
int      log_read(uint16_t idx, log_rec_t *out);
int      log_clear(void);

#endif
