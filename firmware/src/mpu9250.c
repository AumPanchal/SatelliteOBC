#include "mpu9250.h"
#include "board.h"

#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_CFG2  0x1D
#define REG_ACCEL_XOUT  0x3B
#define REG_SIG_RESET   0x68
#define REG_USER_CTRL   0x6A
#define REG_PWR_MGMT1   0x6B
#define REG_WHO_AM_I    0x75

static int wr(spi_dev_t d, uint8_t reg, uint8_t val)
{
    int e = 0;
    spi_begin(d);
    e |= spi_xfer(reg & 0x7F, 0);
    e |= spi_xfer(val, 0);
    spi_end(d);
    return e;
}

static int rd(spi_dev_t d, uint8_t reg, uint8_t *buf, uint8_t n)
{
    int e = 0;
    spi_begin(d);
    e |= spi_xfer(reg | 0x80, 0);
    for (uint8_t i = 0; i < n; i++) {
        e |= spi_xfer(0xFF, &buf[i]);
    }
    spi_end(d);
    return e;
}

int mpu_init(spi_dev_t d)
{
    uint8_t id = 0;
    int e = 0;

    e |= wr(d, REG_PWR_MGMT1, 0x80);      // device reset
    delay_ms(100);
    e |= wr(d, REG_SIG_RESET, 0x07);      // reset sensor paths
    delay_ms(100);
    e |= wr(d, REG_USER_CTRL, 0x10);      // disable I2C, SPI only
    e |= wr(d, REG_PWR_MGMT1, 0x01);      // wake up, PLL clock
    delay_ms(10);

    // check the chip answers with the right id
    e |= rd(d, REG_WHO_AM_I, &id, 1);
    if (e || (id != 0x71 && id != 0x73)) return -1;

    e |= wr(d, REG_CONFIG, 0x03);         // gyro low pass 41 Hz
    e |= wr(d, REG_SMPLRT_DIV, 0x09);     // 100 Hz sample rate
    e |= wr(d, REG_GYRO_CFG, 0x08);       // +-500 dps
    e |= wr(d, REG_ACCEL_CFG, 0x08);      // +-4 g
    e |= wr(d, REG_ACCEL_CFG2, 0x03);     // accel low pass 44 Hz
    return e ? -1 : 0;
}

int mpu_read(spi_dev_t d, imu_data_t *out)
{
    uint8_t b[14];
    uint8_t all_ff = 1, all_00 = 1;

    if (rd(d, REG_ACCEL_XOUT, b, 14)) return -1;

    // a dead or disconnected chip reads all 0xFF or all 0x00
    for (uint8_t i = 0; i < 14; i++) {
        if (b[i] != 0xFF) all_ff = 0;
        if (b[i] != 0x00) all_00 = 0;
    }
    if (all_ff || all_00) return -1;

    out->ax = (int16_t)((b[0] << 8) | b[1]);
    out->ay = (int16_t)((b[2] << 8) | b[3]);
    out->az = (int16_t)((b[4] << 8) | b[5]);
    // b[6], b[7] is the internal temperature, not used
    out->gx = (int16_t)((b[8] << 8) | b[9]);
    out->gy = (int16_t)((b[10] << 8) | b[11]);
    out->gz = (int16_t)((b[12] << 8) | b[13]);
    return 0;
}
