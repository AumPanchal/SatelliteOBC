# Satellite On-Board Computer (OBC)

*A fault-tolerant satellite OBC: STM32F4 firmware with a 4-state FSM and watchdog, CRC-16
validated telemetry over UART, a Python ground station with two-way command uplink, and a
2-layer KiCad PCB design.*

**Status: in progress.**

## Roadmap

- [x] **Phase 0** — Interface spec frozen (`docs/interface-spec.md`) [PUSHED]
- [x] **Phase 1** — KiCad 2-layer PCB design (schematic, layout, ERC/DRC, exports)
- [x] **Phase 2** — STM32 firmware (drivers, CRC-16, FSM, watchdog, telemetry, fault log)
- [x] **Phase 3** — Python ground station (live display, CRC validation, command uplink)
- [ ] **Phase 4** — Bench verification (scope UART/I2C decode, ADC calibration)
- [ ] **Phase 5** — Fault-injection demo + test report
- [ ] **Phase 6** — Final documentation pass
