#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// timing
#define TELEMETRY_PERIOD_MS 100   // 10 Hz
#define WDG_TIMEOUT_MS      1000
#define FAULT_DEBOUNCE      3     // bad cycles in a row before a fault
#define BUS_TIMEOUT_MS      5

// uart
#define UART_BAUD 115200

// i2c addresses
#define TMP117_ADDR 0x48          // ADD0 -> GND
#define INA226_ADDR 0x40          // A0, A1 -> GND

// health limits
#define TEMP_MIN_CDEG      (-2000)   // -20.00 C
#define TEMP_MAX_CDEG      8500      // 85.00 C
#define BUS_MV_MIN         3000
#define BUS_MV_MAX         3600
#define CURRENT_MAX_MA     500
#define IMU_ACCEL_DIFF_MAX 4096      // 0.5 g at +-4 g range
#define IMU_GYRO_DIFF_MAX  3275      // 50 dps at +-500 dps range

// fsm states
typedef enum {
    ST_BOOT = 0,
    ST_SELF_TEST,
    ST_NOMINAL,
    ST_SAFE_MODE
} obc_state_t;

// fault codes saved to flash
enum {
    FAULT_NONE         = 0x00,
    FAULT_WDG_RESET    = 0x01,
    FAULT_CLOCK        = 0x02,
    FAULT_IMU1_COMM    = 0x10,
    FAULT_IMU2_COMM    = 0x11,
    FAULT_IMU_MISMATCH = 0x12,
    FAULT_TEMP_COMM    = 0x20,
    FAULT_TEMP_RANGE   = 0x21,
    FAULT_PWR_COMM     = 0x30,
    FAULT_PWR_VOLTAGE  = 0x31,
    FAULT_PWR_CURRENT  = 0x32,
    FAULT_FLASH_COMM   = 0x40,
    FAULT_GROUND_CMD   = 0x50
};

// status flag bits (1 = healthy)
#define FLAG_IMU1  0x01
#define FLAG_IMU2  0x02
#define FLAG_TEMP  0x04
#define FLAG_PWR   0x08
#define FLAG_FLASH 0x10
#define FLAG_CLOCK 0x20

// packet framing: AA 55 | TYPE | SEQ | LEN | PAYLOAD | CRC_H CRC_L
#define SYNC1 0xAA
#define SYNC2 0x55

// packet types sent by the OBC
#define PKT_TELEMETRY 0x01
#define PKT_ACK       0x02
#define PKT_NACK      0x03
#define PKT_LOG_ENTRY 0x04

// commands sent by the ground station
#define CMD_PING      0x10
#define CMD_GOTO_SAFE 0x11
#define CMD_RECOVER   0x12
#define CMD_GET_LOG   0x13
#define CMD_CLEAR_LOG 0x14
#define CMD_REBOOT    0x15

// nack reasons
#define NACK_BAD_CRC     1
#define NACK_UNKNOWN_CMD 2
#define NACK_BAD_STATE   3
#define NACK_BAD_LEN     4
#define NACK_NO_ENTRY    5
#define NACK_FLASH       6

#endif
