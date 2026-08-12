# Satellite OBC — Interface Specification

**Status:** Frozen — firmware and ground station build against this.
**Rev:** B
**Scope:** Defines the FSM, the telemetry/command wire formats, CRC parameters, and the
fault-code table. Once frozen, the firmware track and the Python ground-station track can
proceed independently because both sides agree on the bytes on the wire.

---

## 1. Hardware context (what this spec assumes)

| Subsystem            | Design intent (PCB)        | Demo implementation (dev board)           |
|----------------------|----------------------------|-------------------------------------------|
| MCU                  | STM32F405 (Cortex-M4)      | Nucleo-F411RE (Cortex-M4)                 |
| Redundant sensing    | Dual IMU                   | Dual **thermal**: MPU-6050 on-die + DHT11 |
| Thermal              | TMP117                     | Covered by MPU on-die temp                |
| Power monitor        | INA226 + shunt (I2C)       | MCU ADC: voltage divider + shunt          |
| Fault log            | External SPI NOR (W25Q)    | Internal flash sector                     |
| Telemetry link       | UART                       | USART2 -> ST-LINK USB VCP -> PC           |

The redundancy architecture (two independent sources, disagreement detection, failover to
the healthy source, fault flag) is identical regardless of whether it runs on IMUs or
thermal sensors.

---

## 2. Link parameters

| Parameter   | Value                          |
|-------------|--------------------------------|
| Baud rate   | 115200                         |
| Frame bits  | 8 data, no parity, 1 stop (8N1)|
| Telemetry rate (NOMINAL) | 10 Hz             |
| Telemetry rate (SAFE)    | 1 Hz              |
| Byte order  | Little-endian                  |

**Physical link (Nucleo-F411RE):** telemetry/command UART is **USART2** on **PA2 (TX) /
PA3 (RX)**, which is bridged by the onboard ST-LINK to a USB Virtual COM Port. The Python
ground station opens that COM port at 115200 8N1. A single USB cable carries both firmware
flashing (SWD) and the telemetry link.

Bandwidth check: 32-byte telemetry frame at 10 Hz = 320 B/s ~= 2.6 kbit/s of payload,
trivial at 115200 baud. Headroom is for command traffic and retries.

---

## 3. Finite State Machine

Four states. Transitions are triggered by sensor health, ground commands, and reset cause.

### States
| Code | State     | Behavior                                                                   |
|------|-----------|----------------------------------------------------------------------------|
| 0x00 | INIT      | Power-on self-test of sensors; check reset cause; route to NOMINAL or SAFE |
| 0x01 | NOMINAL   | Sample sensors, downlink @10 Hz, accept commands, kick watchdog            |
| 0x02 | SAFE      | Downlink @1 Hz, fault logged, watchdog still kicked, await ground          |
| 0x03 | RECOVERY  | Attempt re-init of failed subsystem; return to NOMINAL or fall to SAFE     |

### Transitions
| From      | Trigger                                              | To        | Action                          |
|-----------|------------------------------------------------------|-----------|---------------------------------|
| (boot)    | Reset cause = watchdog                               | INIT      | Log 0x30                        |
| (boot)    | Reset cause = brownout                               | INIT      | Log 0x31                        |
| INIT      | All sensors pass self-test, no logged boot fault     | NOMINAL   | —                               |
| INIT      | Any sensor fails self-test OR boot fault logged      | SAFE      | Log relevant fault              |
| NOMINAL   | Temp sources disagree > threshold                    | SAFE      | Log 0x12, fail over to healthy  |
| NOMINAL   | Sensor read fail / out of range                      | SAFE      | Log 0x10/0x11/0x20              |
| NOMINAL   | Command SET_MODE(SAFE)                               | SAFE      | —                               |
| SAFE      | Command SET_MODE(RECOVERY)                           | RECOVERY  | —                               |
| SAFE      | Command SET_MODE(NOMINAL) (manual override)          | NOMINAL   | Clear active fault flag         |
| RECOVERY  | Subsystem re-init succeeds                           | NOMINAL   | —                               |
| RECOVERY  | Re-init fails (retry count exceeded)                 | SAFE      | Log relevant fault              |

**Disagreement threshold (thermal demo):** sources differ by more than **5.0 °C** for
**3 consecutive samples** -> declare 0x12. (Tune during bring-up.)

---

## 4. Frame structure (common)

All frames share a sync prefix so the receiver can resynchronize after a dropped byte.

```
+--------+--------+ ... payload ... +--------+--------+
| 0xAA   | 0x55   |     (varies)    | CRC_LO | CRC_HI |
+--------+--------+-----------------+--------+--------+
  sync0    sync1                       CRC-16 (LE)
```

- **Sync:** `0xAA 0x55` (2 bytes), fixed.
- **CRC-16:** 2 bytes, little-endian, computed over **every byte after the sync** (i.e.,
  from the length/opcode byte through the end of payload). Sync bytes are NOT included in
  the CRC.

---

## 5. Telemetry frame (OBC -> ground)

```
Offset  Size  Field            Type    Notes
------  ----  ---------------  ------  ---------------------------------------
  0      1    SYNC0            u8      0xAA
  1      1    SYNC1            u8      0x55
  2      1    LEN              u8      payload length in bytes (= 27)
  3      4    timestamp_ms     u32     ms since boot
  7      1    state            u8      FSM state code (see sec 3)
  8      2    temp_A_centi     i16     MPU on-die temp, degC x100
 10      2    temp_B_centi     i16     DHT11 temp, degC x100 (last held value)
 12      1    temp_agree       u8      1 = sources agree, 0 = disagree
 13      2    accel_x          i16     raw IMU
 15      2    accel_y          i16     raw IMU
 17      2    accel_z          i16     raw IMU
 19      2    gyro_x           i16     raw IMU
 21      2    gyro_y           i16     raw IMU
 23      2    gyro_z           i16     raw IMU
 25      2    voltage_mv       u16     bus voltage, millivolts (ADC)
 27      2    current_ma       i16     bus current, milliamps (ADC shunt, approx)
 29      1    last_fault       u8      most recent fault code (see sec 7)
 30      2    CRC16            u16     little-endian, over bytes [2..29]
------  ----
Total frame size = 32 bytes
```

`LEN` counts only the payload (offsets 3..29) = 27 bytes. CRC is computed over LEN + payload
(offsets 2..29).

---

## 6. Command frame (ground -> OBC) and replies

### Command frame
```
Offset  Size  Field      Type   Notes
------  ----  ---------  -----  -------------------------------
  0      1    SYNC0      u8     0xAA
  1      1    SYNC1      u8     0x55
  2      1    OPCODE     u8     see table below
  3      1    ARG_LEN    u8     number of argument bytes (0..N)
  4      N    ARGS       u8[]   opcode-specific
 4+N     2    CRC16      u16    little-endian, over bytes [2 .. 3+N]
```

### Opcodes
| Opcode | Name            | Args                         | Effect                                    |
|--------|-----------------|------------------------------|-------------------------------------------|
| 0x01   | PING            | none                         | OBC replies ACK                           |
| 0x02   | SET_MODE        | 1 byte = target state code   | Request FSM transition (validated)        |
| 0x03   | REQ_SNAPSHOT    | none                         | OBC sends one telemetry frame immediately |
| 0x04   | CLEAR_FAULT_LOG | none                         | Erase fault-log sector; reset last_fault  |
| 0x05   | SOFT_RESET      | none                         | OBC performs a software reset             |

### Reply frame (OBC -> ground, in response to a command)
```
Offset  Size  Field      Type   Notes
------  ----  ---------  -----  ----------------------------------
  0      1    SYNC0      u8     0xAA
  1      1    SYNC1      u8     0x55
  2      1    STATUS     u8     0x06 = ACK, 0x15 = NACK
  3      1    OPCODE     u8     opcode being acknowledged
  4      2    CRC16      u16    little-endian, over bytes [2..3]
```

NACK is sent when: CRC fails (log 0x40), opcode unknown (log 0x41), or a requested
SET_MODE transition is not allowed from the current state.

---

## 7. Fault-code table

| Code | Name             | Meaning                                            |
|------|------------------|----------------------------------------------------|
| 0x00 | NONE             | No active fault                                    |
| 0x10 | TEMP_A_FAIL      | Sensor A (MPU temp) read fail / out of range       |
| 0x11 | TEMP_B_FAIL      | Sensor B (DHT11) read fail / out of range          |
| 0x12 | TEMP_DISAGREE    | Thermal sources differ beyond threshold            |
| 0x20 | IMU_FAIL         | IMU read fail / WHO_AM_I mismatch                  |
| 0x30 | WATCHDOG_RESET   | Last reset was watchdog-induced                    |
| 0x31 | BROWNOUT         | Brownout / low-voltage reset detected              |
| 0x40 | CMD_BAD_CRC      | Received command failed CRC check                  |
| 0x41 | CMD_UNKNOWN_OP   | Received command had an unknown opcode             |

Fault-log record format (written to flash on each fault):
```
+----------+----------+--------+
| ts_ms u32| code u8  | pad u8 |   (6 bytes per record, append-only)
+----------+----------+--------+
```

**Storage location (Nucleo-F411RE):** the fault log is written to **flash sector 7**
(128 KB, base address **0x08060000**) — the last sector, well clear of the vector table
and program code in the low sectors. Records are appended within the sector; CLEAR_FAULT_LOG
erases the entire sector (the F411 erases at sector granularity). Never log to a sector the
CPU is executing from.

---

## 8. CRC-16 definition

Both firmware and ground station MUST use identical parameters.

| Parameter   | Value                |
|-------------|----------------------|
| Algorithm   | CRC-16/CCITT (XModem)|
| Polynomial  | 0x1021               |
| Init        | 0xFFFF               |
| Reflect in  | false                |
| Reflect out | false                |
| XOR out     | 0x0000               |
| Width       | 16 bits              |

**Test vector:** input ASCII `"123456789"` -> CRC = **0x29B1**.
Verify both implementations against this before trusting any frame.

> Note: this is a *software* CRC. The STM32 F1/F4 hardware CRC unit is fixed at 32-bit
> (polynomial 0x04C11DB7) and cannot produce this CRC-16, so it is implemented in code
> (table-driven recommended) on the firmware side.

---

## 9. Resynchronization rule (ground station)

Read byte-by-byte. State machine:
1. Hunt for `0xAA`; if next byte is `0x55`, sync acquired.
2. Read LEN, then LEN payload bytes, then 2 CRC bytes.
3. Validate CRC over LEN+payload. If valid, dispatch frame and increment `frames_ok`.
4. If invalid, discard, increment `frames_rejected`, and return to step 1.

`frames_rejected / (frames_ok + frames_rejected)` is the link-quality metric shown on the
ground display.
