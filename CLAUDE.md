# Swarm C2 TDMA firmware

Fork of mLRS for the Matek mR900-30 (STM32G431KB, SX1262 + TCXO, SE2435L PA).
Goal: beacon-synchronised TDMA, 1 ground node + up to 20 drone nodes.
Spec: docs/phases-2-4.md. Work only on the phase and task named in the session.

## Rules
- Keep the mLRS hardware layer and SX126x driver; put new code in tdma/ (timebase, radio, tdma, dutycycle, dbg, config).
- All times are uint32_t microseconds from TIM2 at 1 MHz; compare with signed differences.
- TIM2 runs from the STM32 clock, not the SX1262 TCXO; drift estimates refer to that clock.
- Slots are numbered 1-20; slot n starts at TDMA_SLOT1_START_US + (n - 1) * TDMA_SLOT_US.
- Beacon RX timeout must cover the lead plus ~21 ms to HeaderValid (TDMA_BEACON_RX_TIMEOUT_US); stop it on header detection, not preamble.
- Never transmit outside the node's own slots. The ground node listens one full frame after boot and rejoins an existing phase from drone slot packets before it beacons.
- TX power is capped at 24 dBm in code. Never remove the cap or the duty-cycle limiter. The field limit later is 27 dBm ERP including antenna gain; 30 dBm is never allowed.
- Constants marked "measured" must come from logged measurements, never estimates.
- Every task ends with: build passes, a short test procedure, and a CSV log format.
- Ask before changing pin assignments, clock config or interrupt priorities.

## Test docs
- docs/prereq-link-test.md (done), docs/phase2-timing-test.md (tasks 2.1-2.4); `python3 tools/tdma_phase2_stats.py ground.csv drone.csv`.

## Build and flash
- One-time setup: `git submodule update --init --recursive`, then `python3 tools/run_copy_st_drivers.py -silent`. Needs `arm-none-eabi-gcc` on PATH.
- Build: `python3 tools/run_make_tdma.py --role ground` or `--role drone --node N` (`--all` builds ground + drones 1-3; `--power` 17..24 dBm, default 17). Output: `tools/build/firmware/tdma-*.hex` and `.bin`.
- The build takes Core, clocks, ST drivers and the USB device library from `tx-matek-mr900-30-g431kb` (same board, USB-C enabled) and the pin hal from `rx-hal-matek-mr900-30-g431kb.h`. It compiles only the mLRS HAL, sx12xx-lib SX126x driver, stm32ll-lib, the USB CDC device and `mLRS/tdma/`; no mLRS link logic.
- Flash by DFU: hold the button while connecting USB, then STM32CubeProgrammer (USB) erase + program the .hex, or `dfu-util -a 0 -s 0x08000000:leave -D <file>.bin`. Role and node ID are compile-time defines: TDMA_ROLE, TDMA_NODE_ID.

## Pins (mR900-30, from the mLRS hal)
- SX1262: SPI1 (PA5/6/7), NSS PA4, DIO1 PA15, BUSY PB5, RESET PB6, TX_EN PB7, RX_EN PB0.
- USB-C (PA11/PA12): CSV log as a USB CDC port (/dev/ttyACM*), also powers the board.
- DIO1 PA15 = TIM2_CH1 input capture (timebase.h); `--dio1-exti` falls back to EXTI15.
- Interrupt priorities: TIM2 1, UART1 11, USB 14 (USBD_IRQ_PRIORITY set by run_make_tdma.py; stock mLRS keeps 0), SysTick 15.
- TX1/RX1 (PA9/PA10): same CSV log, 115200 8N1 (optional). TX2 (PB3): debug pulses. LR1 (PA3): TX-window pin.
- LEDs: green PA1, red PA0. Button PB8. Fan PA8 (off).
