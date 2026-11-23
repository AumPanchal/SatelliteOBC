#include <string.h>
#include "stm32f4xx.h"
#include "config.h"
#include "board.h"
#include "spi.h"
#include "i2c.h"
#include "comm.h"
#include "mpu9250.h"
#include "tmp117.h"
#include "ina226.h"
#include "w25q32.h"
#include "fault_log.h"

static obc_state_t state = ST_BOOT;
static telemetry_t tm;
static uint16_t active_fault = FAULT_NONE;
static uint8_t  bad_count;
static uint8_t  wd_reset, clock_ok, flash_ok;
static uint32_t next_ms;

// keep only the first fault found
static void note(uint16_t *fault, uint16_t code)
{
    if (*fault == FAULT_NONE) *fault = code;
}

// log the fault and switch to safe mode
static void enter_safe(uint16_t code, uint16_t detail)
{
    active_fault = code;
    if (flash_ok && log_write(code, (uint8_t)state, tm.flags, detail)) {
        flash_ok = 0;                         // flash stopped working
    }
    state = ST_SAFE_MODE;
    bad_count = 0;
}

static uint16_t absdiff(int16_t a, int16_t b)
{
    int32_t d = (int32_t)a - b;
    if (d < 0) d = -d;
    return (d > 65535) ? 65535 : (uint16_t)d;
}

// returns 0 if both IMUs agree, otherwise the size of the difference
static uint16_t imu_mismatch(const imu_data_t *a, const imu_data_t *b)
{
    uint16_t d;
    d = absdiff(a->ax, b->ax); if (d > IMU_ACCEL_DIFF_MAX) return d;
    d = absdiff(a->ay, b->ay); if (d > IMU_ACCEL_DIFF_MAX) return d;
    d = absdiff(a->az, b->az); if (d > IMU_ACCEL_DIFF_MAX) return d;
    d = absdiff(a->gx, b->gx); if (d > IMU_GYRO_DIFF_MAX) return d;
    d = absdiff(a->gy, b->gy); if (d > IMU_GYRO_DIFF_MAX) return d;
    d = absdiff(a->gz, b->gz); if (d > IMU_GYRO_DIFF_MAX) return d;
    return 0;
}

// BOOT: bring up the mcu peripherals one at a time
static void do_boot(void)
{
    wd_reset = reset_by_watchdog();
    clock_ok = (clock_init() == 0) ? 1 : 0;
    systick_init();
    gpio_init();
    uart_init();
    spi_init();
    i2c_init();
    wdg_init();
    state = ST_SELF_TEST;
}

// SELF_TEST: check every chip answers
static void do_self_test(void)
{
    uint16_t fault = FAULT_NONE;

    active_fault = FAULT_NONE;

    // flash first so faults can be logged
    flash_ok = (w25_check() == 0 && log_init() == 0) ? 1 : 0;
    if (!flash_ok) note(&fault, FAULT_FLASH_COMM);

    if (!clock_ok)                note(&fault, FAULT_CLOCK);
    if (mpu_init(DEV_IMU1))       note(&fault, FAULT_IMU1_COMM);
    if (mpu_init(DEV_IMU2))       note(&fault, FAULT_IMU2_COMM);
    if (tmp117_init())            note(&fault, FAULT_TEMP_COMM);
    if (ina226_init())            note(&fault, FAULT_PWR_COMM);
    if (wd_reset)                 note(&fault, FAULT_WDG_RESET);
    wd_reset = 0;

    if (fault) {
        enter_safe(fault, 0);
        return;
    }
    next_ms = millis() + TELEMETRY_PERIOD_MS;
    state = ST_NOMINAL;
}

// read all sensors and return the first fault found
static uint16_t sample_and_check(uint16_t *detail)
{
    uint16_t fault = FAULT_NONE;
    int16_t  t;
    uint16_t v;
    int16_t  i;
    int imu1_ok, imu2_ok;

    *detail = 0;
    tm.flags = clock_ok ? FLAG_CLOCK : 0;

    // imus
    imu1_ok = (mpu_read(DEV_IMU1, &tm.imu1) == 0);
    imu2_ok = (mpu_read(DEV_IMU2, &tm.imu2) == 0);
    if (imu1_ok) tm.flags |= FLAG_IMU1; else note(&fault, FAULT_IMU1_COMM);
    if (imu2_ok) tm.flags |= FLAG_IMU2; else note(&fault, FAULT_IMU2_COMM);

    // compare the two imus
    if (imu1_ok && imu2_ok) {
        uint16_t d = imu_mismatch(&tm.imu1, &tm.imu2);
        if (d) {
            note(&fault, FAULT_IMU_MISMATCH);
            *detail = d;
        }
    }

    // temperature
    if (tmp117_read(&t) == 0) {
        tm.temp_cdeg = t;
        tm.flags |= FLAG_TEMP;
        if (t < TEMP_MIN_CDEG || t > TEMP_MAX_CDEG) {
            note(&fault, FAULT_TEMP_RANGE);
            *detail = (uint16_t)t;
        }
    } else {
        note(&fault, FAULT_TEMP_COMM);
    }

    // voltage and current
    if (ina226_read(&v, &i) == 0) {
        tm.bus_mv = v;
        tm.current_ma = i;
        tm.flags |= FLAG_PWR;
        if (v < BUS_MV_MIN || v > BUS_MV_MAX) {
            note(&fault, FAULT_PWR_VOLTAGE);
            *detail = v;
        } else if (i > CURRENT_MAX_MA || i < -CURRENT_MAX_MA) {
            note(&fault, FAULT_PWR_CURRENT);
            *detail = (uint16_t)i;
        }
    } else {
        note(&fault, FAULT_PWR_COMM);
    }

    // flash
    if (w25_check() == 0) tm.flags |= FLAG_FLASH; else note(&fault, FAULT_FLASH_COMM);

    return fault;
}

static void send_ack(uint8_t seq, uint8_t cmd)
{
    comm_send(PKT_ACK, seq, &cmd, 1);
}

static void send_nack(uint8_t seq, uint8_t cmd, uint8_t why)
{
    uint8_t p[2] = { cmd, why };
    comm_send(PKT_NACK, seq, p, 2);
}

// handle one command from the ground station
static void handle_cmd(const frame_t *f)
{
    switch (f->type) {
    case CMD_PING:
        send_ack(f->seq, f->type);
        break;

    case CMD_GOTO_SAFE:
        if (state != ST_SAFE_MODE) enter_safe(FAULT_GROUND_CMD, 0);
        send_ack(f->seq, f->type);
        break;

    case CMD_RECOVER:
        // only leave safe mode by re-running the self test
        if (state != ST_SAFE_MODE) {
            send_nack(f->seq, f->type, NACK_BAD_STATE);
        } else {
            send_ack(f->seq, f->type);
            state = ST_SELF_TEST;
        }
        break;

    case CMD_GET_LOG: {
        if (f->len != 2) {
            send_nack(f->seq, f->type, NACK_BAD_LEN);
            break;
        }
        uint16_t idx = (uint16_t)(f->payload[0] | (f->payload[1] << 8));
        log_rec_t r;
        if (log_read(idx, &r)) {
            send_nack(f->seq, f->type, NACK_NO_ENTRY);
            break;
        }
        uint8_t p[2 + sizeof(log_rec_t)];
        p[0] = f->payload[0];
        p[1] = f->payload[1];
        memcpy(&p[2], &r, sizeof(log_rec_t));
        comm_send(PKT_LOG_ENTRY, f->seq, p, sizeof(p));
        break;
    }

    case CMD_CLEAR_LOG:
        if (log_clear()) send_nack(f->seq, f->type, NACK_FLASH);
        else send_ack(f->seq, f->type);
        break;

    case CMD_REBOOT:
        send_ack(f->seq, f->type);
        uart_flush();
        NVIC_SystemReset();
        break;

    default:
        send_nack(f->seq, f->type, NACK_UNKNOWN_CMD);
        break;
    }
}

// NOMINAL and SAFE_MODE: commands every pass, sensors + telemetry at 10 Hz
static void run_cycle(void)
{
    frame_t f;
    int r = comm_poll(&f);
    if (r == 1) handle_cmd(&f);
    else if (r == -1) send_nack(f.seq, 0, NACK_BAD_CRC);

    // wait for the next 100 ms slot
    if ((int32_t)(millis() - next_ms) < 0) return;
    next_ms += TELEMETRY_PERIOD_MS;
    if ((int32_t)(millis() - next_ms) >= 0) next_ms = millis() + TELEMETRY_PERIOD_MS;

    uint16_t detail;
    uint16_t fault = sample_and_check(&detail);

    // only nominal mode can trigger a new fault
    if (state == ST_NOMINAL) {
        if (fault) {
            if (++bad_count >= FAULT_DEBOUNCE) enter_safe(fault, detail);
        } else {
            bad_count = 0;
        }
    }

    // send telemetry
    tm.time_ms   = millis();
    tm.state     = (uint8_t)state;
    tm.fault     = (uint8_t)active_fault;
    tm.log_count = log_count();
    comm_send_telemetry(&tm);
}

int main(void)
{
    for (;;) {
        switch (state) {
        case ST_BOOT:      do_boot();      break;
        case ST_SELF_TEST: do_self_test(); break;
        case ST_NOMINAL:
        case ST_SAFE_MODE: run_cycle();    break;
        }
        wdg_kick();   // only reached if the loop is still running
    }
}
