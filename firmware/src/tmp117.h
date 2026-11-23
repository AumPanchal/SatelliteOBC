#ifndef TMP117_H
#define TMP117_H

#include <stdint.h>

int tmp117_init(void);
int tmp117_read(int16_t *centi_deg);   // temperature in 0.01 C

#endif
