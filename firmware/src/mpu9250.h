#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "spi.h"

typedef struct {
    int16_t ax, ay, az;   // +-4 g    -> 8192 LSB/g
    int16_t gx, gy, gz;   // +-500 dps -> 65.5 LSB/dps
} imu_data_t;

int mpu_init(spi_dev_t dev);
int mpu_read(spi_dev_t dev, imu_data_t *out);

#endif
