# Phase 2 timing test (tasks 2.1–2.4)

Firmware: `mLRS/tdma/` (timebase.h, radio.h, tdma-main.cpp), built with `tools/run_make_tdma.py`.

| Task | Where | What it does |
| --- | --- | --- |
| 2.1 Timebase | `timebase.h` | TIM2 free-running 32-bit at 1 MHz (170 MHz / 170); `tb_now()`, `tb_diff()` (signed), one-shot `tb_at(t, fn)` on CH3, non-blocking debug pulses on CH4 |
| 2.2 DIO1 | `timebase.h` | DIO1 is PA15 = TIM2_CH1 (AF1): hardware input capture, rising edge. Fallback `--dio1-exti` uses EXTI15 and `tb_now()` in the handler |
| 2.3 HeaderValid | `radio.h` | DIO1 carries HeaderValid, RxDone, TxDone, CrcErr, Timeout. The capture of the first DIO1 edge of a packet is its HeaderValid time; 20 µs pulse |
| 2.4 Scheduled TX | `radio.h` | `radio.TxAt(t_on_air, buf, len)`: STDBY_XOSC + buffer loaded in advance, SetTx issued from the TIM2 interrupt at `t_on_air − T_TX_LATENCY_US`; 40 µs pulse at SetTx, 80 µs at TxDone |

Interrupt priorities (lower = higher): TIM2 1, UART1 11, USB 14 (was 0 in mLRS; overridden only for TDMA builds), SysTick 15.

## What the test firmware does

- Ground (`--role ground`): transmits a 30-byte test packet every 1 000 000 TIM2 µs on a fixed grid, using `TxAt`. 6.2% duty. The spec's 500 ms interval for task 2.5 would be 12.4%, over the limit, so the test uses 1 s.
- Drone (`--role drone --node N`): listens only, never transmits. Logs the HeaderValid capture time of every packet and the interval to the previous one. Any number of drones can listen at once.

## Build

```bash
python3 tools/run_make_tdma.py --all
```

## Procedure

1. Antennas or attenuators fitted on both units. Boards 2 m or more apart if on antennas (RSSI below about −10 dBm, not pinned at −1).
2. Flash the ground image to the ground board (208933A44C45) and the drone image to the drone board (209C33854C45).
3. Both on USB-C. Log each with a wall-clock prefix:

   ```bash
   sudo cat /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_208933A44C45-if00 | ts '%H:%M:%S' | tee ground.csv
   ```

   ```bash
   sudo cat /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_209C33854C45-if00 | ts '%H:%M:%S' | tee drone.csv
   ```

4. Run 15 minutes or more (900+ packets), drone on USB power (not a battery bank).
5. Summarise:

   ```bash
   python3 tools/tdma_phase2_stats.py ground.csv drone.csv
   ```

6. Optional, with a logic analyser on TX2 (debug pin) and LR1 (TX window) of both boards: the ground's 40 µs SetTx pulses are exactly 1 000 000 ticks apart (task 2.1 check: 1.000 s ± 50 ppm); the drone's 20 µs HeaderValid pulses follow the ground's SetTx by the sum T_TX_LATENCY_US + T_RX_HDRVALID_OFFSET_US (task 2.5).

## Pass (this session's tasks)

| Measure | Target |
| --- | --- |
| Ground `late` and `tx_fail` | 0 |
| Ground `settx_err_us` (SetTx interrupt latency) | sd under 5 µs, max under 20 µs |
| Ground `txdone_minus_settx_us` | about 61 700 µs plus TX latency, sd under 20 µs |
| Drone packets received | ≥ 99% |
| Drone `err_us` (interval jitter) | sd under 20 µs |
| Clock error (ppm) | recorded; under ±100 ppm (two ±50 ppm crystals) |

## CSV log format

Every 10 s both nodes print `# alive,uptime_s=...,role=...,node=...,power_dbm=...,tx=...,late=...,rx_ok=...,rx_err=...,tb_us=...`.

Ground:

| Column | Meaning |
| --- | --- |
| event | `tx`, `late` (could not arm in time, period skipped), `tx_fail` (no TxDone) |
| seq | test packet number, one per period |
| t_plan_us | planned on-air time (ground TIM2) |
| t_settx_us | SetTx issued (ground TIM2) |
| t_txdone_us | TxDone, DIO1 capture (ground TIM2) |
| settx_err_us | t_settx − (t_plan − T_TX_LATENCY_US) |
| txdone_minus_settx_us | t_txdone − t_settx |

Drone:

| Column | Meaning |
| --- | --- |
| event | `rx`, `crc_err`, `bad_pkt` |
| seq | test packet number |
| t_hdrvalid_us | HeaderValid, DIO1 capture (drone TIM2) |
| interval_us | t_hdrvalid − previous t_hdrvalid |
| err_us | interval_us − (seq difference × 1 000 000) |
| ppm_x10 | err_us relative to the expected interval, 0.1 ppm; positive = drone clock fast vs ground |
| rssi, snr | dBm, dB (RSSI includes the ~16 dB LNA gain) |
| crc_ok | 1 or 0 |
