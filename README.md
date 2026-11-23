# Satellite Onboard Computer (OBC)

A small onboard computer for a satellite, built around an STM32F411. It reads redundant sensors, checks that they and the system are healthy, reacts to faults, saves fault records to flash, and streams CRC-checked telemetry to a Python ground station over UART.

**Stack:** KiCad, STM32, C, Python, I2C, SPI, UART, CRC-16

```
PCB -> STM32 -> C firmware -> Sensors -> Telemetry -> Python ground station
```

![Schematic](hardware/images/schematic.png)

| PCB layout | 3D render |
|---|---|
| ![PCB layout](hardware/images/pcb_layout.png) | ![3D render](hardware/images/pcb_3d.png) |

## Hardware

2-layer KiCad PCB. Placement and routing were iterated until the board passed ERC/DRC.

| Part | Role | Bus |
|---|---|---|
| STM32F411RET6 | Main processor, 8 MHz crystal, PLL to 84 MHz | n/a |
| 2x MPU-9250 | Redundant IMUs (accel + gyro) | SPI1 |
| TMP117 | Temperature sensor | I2C1 (0x48) |
| INA226 + 0.1 ohm shunt | Voltage and current monitor | I2C1 (0x40) |
| W25Q32JV | SPI flash for the fault log | SPI1 |
| AMS1117-3.3 | 3.3 V regulator | n/a |

### Pin map

| Function | Pins |
|---|---|
| UART (USART2) TX / RX | PA2 / PA3 (header J2) |
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 |
| Chip select IMU1 / IMU2 / flash | PB0 / PB1 / PB2 |
| I2C1 SCL / SDA | PB6 / PB7 (4.7k pullups) |
| SWD debug | PA13 / PA14 (header J1) |

## Firmware

Bare-metal C using CMSIS register access (no HAL), in `firmware/src/`.

| File | Role |
|---|---|
| `main.c` | FSM, fault debounce, command handler |
| `board.c` | Clock, GPIO, SysTick, watchdog |
| `spi.c`, `i2c.c` | Bus drivers with timeouts and bus recovery |
| `mpu9250.c`, `tmp117.c`, `ina226.c` | Sensor drivers |
| `w25q32.c`, `fault_log.c` | SPI flash driver and fault log |
| `comm.c`, `crc16.c` | UART, framing, CRC-16 |
| `config.h` | Thresholds, fault codes, protocol IDs |

### State machine

| State | Behavior |
|---|---|
| BOOT | Bring up clock, GPIO, UART, SPI, I2C one at a time, then start the watchdog |
| SELF_TEST | Check flash, both IMUs, TMP117, INA226 and the clock all respond |
| NOMINAL | Sample sensors and send telemetry at 10 Hz, accept commands |
| SAFE_MODE | A fault was found: it is saved to flash, telemetry continues with the fault code, and the OBC waits for the ground station |

Transitions:
- BOOT -> SELF_TEST always.
- SELF_TEST -> NOMINAL if every check passes, otherwise -> SAFE_MODE.
- NOMINAL -> SAFE_MODE when a fault is seen 3 cycles in a row, or on a ground command.
- SAFE_MODE -> SELF_TEST only on a `RECOVER` command from the ground.

### Health checks (NOMINAL)

- Each sensor still answers on its bus.
- IMU1 and IMU2 agree within a threshold (redundancy check).
- Temperature is inside its limits.
- Bus voltage and current are inside their limits.
- Flash JEDEC ID reads correctly.

Limits are in `config.h`. The INA226 limits assume it monitors the 3.3 V rail, so adjust them if it measures something else.

### Watchdog

The independent watchdog (IWDG) has a timeout of about 1 s and is only kicked when the main loop completes. A watchdog reset is detected on the next boot, logged as a fault, and puts the OBC in SAFE_MODE.

### Fault log

Records are 16 bytes, stored in the first 4 KB sector of the W25Q32 (256 entries, then the sector is erased and reused). Each record has a magic number, timestamp, fault code, state, sensor flags, detail value and CRC-16, so records cut short by a power loss are rejected.

| Code | Fault |
|---|---|
| 0x01 | Watchdog reset |
| 0x02 | Crystal / clock failure |
| 0x10 / 0x11 | IMU1 / IMU2 communication |
| 0x12 | IMU1 and IMU2 disagree |
| 0x20 / 0x21 | Temperature sensor communication / out of range |
| 0x30 / 0x31 / 0x32 | Power monitor communication / voltage / current |
| 0x40 | Flash communication |
| 0x50 | Ground commanded safe mode |

### Building

This repo holds the source files only. Create an STM32CubeIDE project for the STM32F411RE and add `firmware/src` as a source folder. CubeIDE supplies the startup file, linker script and CMSIS headers. The sources compile cleanly with `arm-none-eabi-gcc -Wall -Wextra`.

## Communication protocol

Serial settings: 115200 baud, 8N1. Telemetry is sent at 10 Hz.

### Frame format

```
AA 55 | TYPE | SEQ | LEN | PAYLOAD (LEN bytes) | CRC_H CRC_L
```

The CRC is CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF) over TYPE, SEQ, LEN and PAYLOAD, sent high byte first. Test vector: `"123456789"` gives `0x29B1`. Payload fields are little-endian.

### OBC to ground

| Type | Meaning |
|---|---|
| 0x01 | Telemetry (39-byte payload) |
| 0x02 | ACK, payload is the command being acknowledged |
| 0x03 | NACK, payload is the command and a reason |
| 0x04 | Log entry, payload is the index (2 bytes) plus the 16-byte record |

Telemetry payload, Python `struct` format `<IBBB6h6hhHhH`:
time_ms, state, fault, flags, IMU1 (ax ay az gx gy gz), IMU2 (same), temperature in 0.01 C, bus mV, current mA, log count.

Accelerometer scale is 8192 LSB/g (+-4 g). Gyro scale is 65.5 LSB/dps (+-500 dps).

### Ground to OBC

| Command | Name | Payload |
|---|---|---|
| 0x10 | PING | none |
| 0x11 | GOTO_SAFE | none |
| 0x12 | RECOVER | none |
| 0x13 | GET_LOG | entry index (2 bytes) |
| 0x14 | CLEAR_LOG | none |
| 0x15 | REBOOT | none |

NACK reasons: 1 bad CRC, 2 unknown command, 3 wrong state, 4 bad length, 5 no such log entry, 6 flash error.

## Ground station

`ground_station/ground_station.py` decodes telemetry, shows a live status line, counts dropped packets and CRC errors, and can save everything to CSV. Commands are sent with ACK/NACK handling and automatic retries.

```
pip install pyserial
python ground_station/ground_station.py COM3
python ground_station/ground_station.py /dev/ttyUSB0 --csv flight.csv
```

Connect a USB-UART adapter to header J2 (TX, RX, GND). At the `obc>` prompt:

| Command | Action |
|---|---|
| `ping` | Check the link |
| `safe` | Force safe mode |
| `recover` | Leave safe mode (re-runs the self test) |
| `log N` / `logs` | Read one / all fault log entries |
| `clear` | Erase the fault log |
| `reboot` | Reboot the OBC |
| `imu` | Show full IMU data from the last packet |
| `watch` | Toggle the live status line |

Quick demo: run `safe`, then `logs` to see the fault saved to flash, then `recover` to return to NOMINAL.

## Repo layout

```
SatelliteOBC/
├── firmware/src/          C firmware
├── ground_station/        Python ground station
├── hardware/              KiCad project and images
│   └── images/
└── docs/                  Project writeup
```

## Development notes

- Peripherals were brought up one at a time (UART, then SPI, then I2C) to isolate timing and initialization bugs before integrating everything.
- The hard part of the PCB was routing: several interfaces leave the STM32 for different peripherals, so one badly placed part could cause congestion elsewhere. Placement was adjusted and traces rerouted repeatedly to meet the design rules.