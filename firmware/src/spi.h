#ifndef SPI_H
#define SPI_H

#include <stdint.h>

typedef enum { DEV_IMU1 = 0, DEV_IMU2, DEV_FLASH } spi_dev_t;

void spi_init(void);
void spi_begin(spi_dev_t d);          // set speed and pull CS low
void spi_end(spi_dev_t d);            // CS high
int  spi_xfer(uint8_t tx, uint8_t *rx);

#endif
