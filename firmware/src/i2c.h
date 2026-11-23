#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);
int  i2c_read16(uint8_t addr, uint8_t reg, uint16_t *val);   // big-endian 16 bit register
int  i2c_write16(uint8_t addr, uint8_t reg, uint16_t val);

#endif
