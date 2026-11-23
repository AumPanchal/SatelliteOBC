#include "fault_log.h"
#include "w25q32.h"
#include "crc16.h"
#include "board.h"

#define LOG_BASE        0x000000UL     // first 4 KB sector of the flash
#define LOG_REC_SIZE    16
#define LOG_MAX_RECORDS 256
#define LOG_MAGIC       0xFA17C0DEUL

_Static_assert(sizeof(log_rec_t) == LOG_REC_SIZE, "log record must be 16 bytes");

static uint16_t count;

int log_init(void)
{
    count = 0;

    // erased flash reads 0xFF, so the first blank magic is the free slot
    for (uint16_t i = 0; i < LOG_MAX_RECORDS; i++) {
        uint8_t m[4];
        if (w25_read(LOG_BASE + i * LOG_REC_SIZE, m, 4)) return -1;
        if (m[0] == 0xFF && m[1] == 0xFF && m[2] == 0xFF && m[3] == 0xFF) break;
        count++;
    }
    return 0;
}

uint16_t log_count(void)
{
    return count;
}

int log_write(uint16_t code, uint8_t state, uint8_t flags, uint16_t detail)
{
    // sector full: erase and start over
    if (count >= LOG_MAX_RECORDS) {
        if (w25_sector_erase(LOG_BASE)) return -1;
        count = 0;
    }

    log_rec_t r;
    r.magic   = LOG_MAGIC;
    r.time_ms = millis();
    r.code    = code;
    r.state   = state;
    r.flags   = flags;
    r.detail  = detail;
    r.crc     = crc16((const uint8_t *)&r, LOG_REC_SIZE - 2);

    uint16_t slot = count++;
    return w25_page_program(LOG_BASE + slot * LOG_REC_SIZE, (const uint8_t *)&r, LOG_REC_SIZE);
}

int log_read(uint16_t idx, log_rec_t *out)
{
    if (idx >= count) return -1;
    if (w25_read(LOG_BASE + idx * LOG_REC_SIZE, (uint8_t *)out, LOG_REC_SIZE)) return -1;

    // reject records that were cut short by a power loss
    if (out->magic != LOG_MAGIC) return -1;
    if (out->crc != crc16((const uint8_t *)out, LOG_REC_SIZE - 2)) return -1;
    return 0;
}

int log_clear(void)
{
    if (w25_sector_erase(LOG_BASE)) return -1;
    count = 0;
    return 0;
}
