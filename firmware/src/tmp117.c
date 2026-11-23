#include "tmp117.h"
#include "i2c.h"
#include "board.h"
#include "config.h"

#define REG_TEMP 0x00
#define REG_ID   0x0F

int tmp117_init(void)
{
    uint16_t id, raw = 0x8000;

    // check the device id
    if (i2c_read16(TMP117_ADDR, REG_ID, &id)) return -1;
    if ((id & 0x0FFF) != 0x117) return -1;

    // wait for the first conversion
    for (int i = 0; i < 30 && raw == 0x8000; i++) {
        delay_ms(10);
        if (i2c_read16(TMP117_ADDR, REG_TEMP, &raw)) return -1;
    }
    return (raw == 0x8000) ? -1 : 0;
}

int tmp117_read(int16_t *centi_deg)
{
    uint16_t raw;
    if (i2c_read16(TMP117_ADDR, REG_TEMP, &raw)) return -1;
    if (raw == 0x8000) return -1;                      // no valid result
    // 1 LSB = 0.0078125 C -> *100 /128 = *25 /32
    *centi_deg = (int16_t)(((int32_t)(int16_t)raw * 25) / 32);
    return 0;
}
