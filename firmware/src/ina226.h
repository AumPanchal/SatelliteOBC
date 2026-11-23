#ifndef INA226_H
#define INA226_H

#include <stdint.h>

int ina226_init(void);
int ina226_read(uint16_t *bus_mv, int16_t *current_ma);

#endif
