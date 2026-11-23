#include "ina226.h"
#include "i2c.h"
#include "board.h"
#include "config.h"

#define REG_CONFIG  0x00
#define REG_BUS_V   0x02
#define REG_CURRENT 0x04
#define REG_CALIB   0x05
#define REG_MFR_ID  0xFE

// shunt is 0.1 ohm, current LSB = 0.1 mA -> cal = 0.00512 / (0.0001 * 0.1)
#define CAL_VALUE 512

int ina226_init(void)
{
    uint16_t id, cal;

    // check manufacturer id
    if (i2c_read16(INA226_ADDR, REG_MFR_ID, &id) || id != 0x5449) return -1;

    if (i2c_write16(INA226_ADDR, REG_CONFIG, 0x4527)) return -1;   // 16x average, continuous
    if (i2c_write16(INA226_ADDR, REG_CALIB, CAL_VALUE)) return -1;

    // confirm the write stuck
    if (i2c_read16(INA226_ADDR, REG_CALIB, &cal) || cal != CAL_VALUE) return -1;

    delay_ms(50);   // first averaged conversion
    return 0;
}

int ina226_read(uint16_t *bus_mv, int16_t *current_ma)
{
    uint16_t v, c;
    if (i2c_read16(INA226_ADDR, REG_BUS_V, &v)) return -1;
    if (i2c_read16(INA226_ADDR, REG_CURRENT, &c)) return -1;

    *bus_mv = (uint16_t)(((uint32_t)v * 5) / 4);   // 1.25 mV per LSB
    *current_ma = (int16_t)c / 10;                 // 0.1 mA per LSB
    return 0;
}
