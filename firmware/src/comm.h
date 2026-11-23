#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "mpu9250.h"

#define RX_MAX_PAYLOAD 32

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[RX_MAX_PAYLOAD];
} frame_t;

typedef struct {
    uint32_t   time_ms;
    uint8_t    state;
    uint8_t    fault;
    uint8_t    flags;
    imu_data_t imu1;
    imu_data_t imu2;
    int16_t    temp_cdeg;
    uint16_t   bus_mv;
    int16_t    current_ma;
    uint16_t   log_count;
} telemetry_t;

void uart_init(void);
void uart_flush(void);
void comm_send(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);
void comm_send_telemetry(const telemetry_t *t);
int  comm_poll(frame_t *f);    // 1 = good frame, -1 = bad crc (seq filled in), 0 = nothing

#endif
