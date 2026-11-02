# Swarm C2 Link — Firmware Phases 2–4 (Matek mR900-30)

Nov 2, 2026 · @geo

## Scope and starting point

Phases 2–4 take four Matek mR900-30 units from basic packet exchange to a working, fault-tolerant beacon-synchronised TDMA frame: one ground node and three drone nodes covering all 20 slots.

| Phase | Delivers | Exit test |
| --- | --- | --- |
| 2. Timing primitives | Microsecond timestamps of beacon reception, scheduled transmit, debug pulses | All nodes agree on frame zero to under 100 µs |
| 3. Beacon and slot scheduler | 2.0 s frame, 20 slots, ground logging, duty-cycle accounting | 1 hour, no collisions, ≥ 99.5% of slot packets received |
| 4. Holdover and robustness | Holdover, silence rule, re-acquisition, drift correction | No out-of-slot transmission in any fault test |

Phases 0–1 are skipped, so these must already work before starting Phase 2:

- [ ] mLRS forked into a Git repo; the `rx-matek-mr900-30-g431kb` target builds from source and flashes by DFU.
- [ ] A stripped firmware that keeps the mLRS hardware layer and SX126x driver but not its link logic.
- [ ] Two units exchanging fixed packets at 869.525 MHz, SF8, 250 kHz, CR 4/5, with RSSI and SNR printed over UART.

If any of these is missing, do it as the first Claude Code session (see the last sections). MAVLink, real packet formats and authentication are out of scope; they arrive in Phases 5–6.

## Bench hardware and RF setup

All four nodes are joined by cable through attenuators and a 4-way splitter, so tests are repeatable and no node overloads another's receiver.

| Node | Role | Slot coverage in Phase 3 |
| --- | --- | --- |
| A | Ground node | Beacon slot |
| B | Drone node | Slots 1, 2, 3 |
| C | Drone node | Slots 9, 10, 11 |
| D | Drone node | Slots 18, 19, 20 |

**RF rig:** each node's IPEX connector goes to an IPEX–SMA pigtail, then a 30 dB SMA attenuator, then one port of a 4-port resistive star divider (rated to 1 GHz or more). Use a resistive divider, not a Wilkinson-type power splitter: between a Wilkinson's output ports the path is its isolation (about 20–25 dB, different for each pair), so node-to-node loss would be uneven. A resistive star gives the same loss, about 9.5 dB, between any two ports. Node-to-node loss is then about 30 + 9.5 + 30 ≈ 70 dB. At the 24 dBm bench cap each receiver sees about −46 dBm (about −53 dBm at the 17 dBm minimum), well below the SX1262's damage level and well above sensitivity. Without a VNA, check the rig with the radios: each node transmits in turn at 24 dBm while the other three log RSSI. All 12 paths should read about −46 dBm and agree within about 3 dB; one path far off means a bad cable, connector or attenuator.

**Other equipment:**

- Each node plugged into the laptop by USB-C: the log arrives as a USB CDC port (`/dev/ttyACM*`) and USB powers the board. Use powered USB ports or a powered hub: at 24 dBm a node can draw several hundred mA. The same log is on TX1/RX1 (115200) if a USB–UART adapter is preferred.
- 8-channel logic analyser on each node's debug pin (see next section) plus one channel spare for a GPS PPS reference.
- No SDR or VNA in these phases. Instead, each node drives a TX-window pin, LR1 (PA3), high while its PA is switched to transmit (set with SX\_TX\_EN, cleared at TxDone or when switching to receive). Four debug pins plus four TX-window pins use all 8 logic-analyser channels, so there is no GPS PPS channel. The radios' own RSSI stands in for a VNA when checking the RF rig.
- Bench supply at 5 V, able to deliver about 1 A per node (Matek quotes 850 mA peak at 30 dBm).
- Supplied heatsinks fitted to every unit.

**Safety rules:**

- Never transmit without an antenna or attenuator chain fitted; the SE2435L amplifier can be damaged into an open port.
- Keep transmit power at or below 24 dBm for all bench work in these phases. Matek lists 17 / 20 / 24 / 27 / 30 dBm at 868 MHz, so 17 dBm is the true minimum and the default for bench runs.
- Firmware must cap power in code so a bad setting cannot select 30 dBm. For later field use, 869.4–869.65 MHz allows 500 mW (27 dBm) ERP including antenna gain, so 30 dBm is never legal in this sub-band.

## Firmware structure and conventions

One firmware image serves both roles, selected by a compile-time role and node ID; the new code sits beside the mLRS hardware layer, not inside it.

| Module | Responsibility |
| --- | --- |
| `timebase` | Free-running 1 MHz, 32-bit hardware timer (TIM2 on the STM32G431); input capture and output compare helpers |
| `radio` | Thin wrapper over the mLRS SX126x driver: configure, preload TX buffer, start RX, read IRQ status, RSSI and SNR |
| `tdma` | Frame state machine: acquisition, tracking, holdover, silent; slot table; next TX time |
| `dutycycle` | Rolling 1-hour airtime accounting; blocks TX above the limit |
| `dbg` | Debug pin pulses and the UART log |
| `config` | Role, node ID, slot list, radio settings |

**Timebase rule:** every timestamp in the system is a `uint32_t` microsecond count from the one timer. Compare times with signed differences so wrap-around (every \~71 minutes) is harmless.

**Starting constants** (tune from measurements, never guess):

```c
#define TDMA_FREQ_HZ            869525000UL  // 869.4-869.65 MHz; 250 kHz BW fills the whole sub-band, no edge margin
#define TDMA_SF                 8
#define TDMA_BW_KHZ             250
#define TDMA_CR                 5            // 4/5
#define TDMA_PREAMBLE_SYMS      8
#define TDMA_TX_POWER_DBM_MAX   24           // bench cap for phases 2-4; field max 27 dBm ERP incl. antenna gain

#define TDMA_FRAME_US           2000000UL
#define TDMA_BEACON_START_US    0
#define TDMA_SLOT1_START_US     150000UL     // slots are numbered 1-20
#define TDMA_SLOT_US            80000UL
#define TDMA_NUM_SLOTS          20
#define TDMA_SPARE_START_US     1750000UL

// Airtime at SF8 / 250 kHz / CR 4/5, explicit header, CRC on:
//   30-byte slot packet ~61.7 ms (18.3 ms guard), 40-byte beacon ~77.1 ms
//   HeaderValid ~20.7 ms after TX start (12.25 preamble + 8 header symbols)
#define TDMA_BEACON_RX_LEAD_US     5000UL   // open RX this early (TRACK)
#define TDMA_BEACON_RX_TIMEOUT_US  35000UL  // lead + ~21 ms to HeaderValid + margin

#define TDMA_HOLDOVER_FRAMES    30           // then go silent
#define TDMA_DUTY_LIMIT_PPM     100000UL     // 10% per hour

// Measured in Phase 2, not guessed:
#define T_RX_HDRVALID_OFFSET_US 0            // TX start -> receiver HeaderValid
#define T_TX_LATENCY_US         0            // SetTx command -> RF on air
```

**Debug pin:** use TX2 (normally RC out) as a GPIO. Pulse widths identify the event on the logic analyser:

| Event | Pulse width |
| --- | --- |
| Frame zero (computed) | 10 µs |
| HeaderValid captured | 20 µs |
| SetTx issued | 40 µs |
| TxDone | 80 µs |

If TX2 cannot be freed, use the LED pins as fallback. The UART log is line-based CSV so it can be captured straight into a file.

## Phase 2: Timing primitives

Phase 2 proves that a drone node can turn a received beacon into a frame-zero time that matches the ground node's to within 100 µs, and that it can start a transmission at a chosen microsecond.

### Tasks

- [ ] **2.1 Timebase.** Configure TIM2 as a free-running 32-bit counter at 1 MHz. Add `tb_now()`, `tb_diff(a, b)` (signed) and an output-compare callback `tb_at(t, fn)`. Check: toggle the debug pin every 1 000 000 ticks and confirm 1.000 s ± 50 ppm on the logic analyser.
- [ ] **2.2 Find DIO1.** From the mLRS hardware header for this target, find the GPIO wired to SX1262 DIO1 and whether it lands on a TIM input-capture channel. If yes, capture in hardware. If not, use EXTI and read `tb_now()` first thing in the handler, then measure the extra latency (see 2.5).
- [ ] **2.3 HeaderValid interrupt.** Set the SX1262 IRQ mask so HeaderValid, RxDone, TxDone, CrcErr and Timeout route to DIO1. On HeaderValid, store the timestamp; on RxDone, read the IRQ status, check CRC, and keep the HeaderValid time with the packet. Pulse the debug pin (20 µs) on each capture.
- [ ] **2.4 Scheduled transmit.** Add `radio_tx_at(t_on_air, buf, len)`: write the payload to the buffer and put the radio in STDBY\_XOSC in advance, then use `tb_at()` to issue SetTx at `t_on_air − T_TX_LATENCY_US`. Pulse the debug pin (40 µs) at SetTx and (80 µs) at TxDone.
- [ ] **2.5 Measure the two offsets.** Node A transmits a test packet every 500 ms using `radio_tx_at`. Node B receives. On the logic analyser, measure:
  - `T_TX_LATENCY_US`: A's SetTx pulse to RF on air (TxDone time minus the calculated airtime). Without an SDR the split between the two offsets cannot be cross-checked, and it does not need to be: slot alignment depends only on their sum, which the logic analyser measures directly as A's SetTx pulse to B's HeaderValid pulse. Record that sum as the primary figure; an error in the split shifts every node's on-air time by the same amount and leaves rx\_offset\_us unchanged.
  - `T_RX_HDRVALID_OFFSET_US`: A's on-air start to B's HeaderValid pulse. Record mean, standard deviation and maximum over at least 1000 packets, and write the means into the constants.
- [ ] **2.6 Frame zero.** On B, compute `frame_zero = t_hdrvalid − T_RX_HDRVALID_OFFSET_US` and pulse the debug pin (10 µs) at that instant using output compare one frame later. A pulses at its own frame zero. Repeat with C and D receiving at the same time.

### Measurement procedure

1. Logic analyser at 10 MS/s or faster, one channel per node's debug pin, common ground.
2. Run 1000 beacons at 2.0 s intervals (about 35 minutes).
3. Export pulse edges to CSV and compute, per drone node, the difference between its frame-zero pulse and A's.
4. Repeat at three received levels (vary the attenuators by ±10 dB) to check the offset does not depend on signal strength.

### Exit criteria

| Measure | Target |
| --- | --- |
| Frame-zero error, mean (after calibration) | Under 20 µs |
| Frame-zero error, standard deviation | Under 20 µs |
| Frame-zero error, worst case over 1000 beacons | Under 100 µs |
| Change in offset across ±10 dB signal level | Under 20 µs |
| SetTx-to-air latency spread | Under 20 µs |

If the worst case exceeds 100 µs, check for interrupt priority clashes with the mLRS USB or UART handlers before widening any guards.

## Phase 3: Beacon and slot scheduler

Phase 3 runs the full 2.0 s frame: node A beacons, nodes B–D acquire it and transmit in their assigned slots, and node A logs every slot packet with its arrival time.

**Duty-cycle catch:** each 62 ms slot packet is 3.1% duty, so one node can legally cover at most 3 slots (9.3%). Seven slots per node would be about 22%, over the 10% limit. The default load test therefore covers 9 slots spread across the frame; a full 20-slot test is a short, cabled-rig-only option.

| Node | Default slots (legal, 9.3% each) | Full-load option (cabled rig only) |
| --- | --- | --- |
| B | 1, 2, 3 | 1–7 |
| C | 9, 10, 11 | 8–14 |
| D | 18, 19, 20 | 15–20 |

The default set exercises the start, middle and end of the frame, where timing errors would show first.

### Tasks

- [ ] **3.1 Slot table.** `config` holds a role, node ID and a 20-bit slot mask. Slot n starts at `TDMA_SLOT1_START_US + (n − 1) × TDMA_SLOT_US` after frame zero.
- [ ] **3.2 Ground node.** After boot, listen for one full frame (2.0 s) before the first beacon. If it hears drone slot packets, rebuild the existing frame zero and counter from their slot number, echoed frame counter and HeaderValid time, and carry on in that phase with the counter still rising. Only if it hears nothing does it start a new phase. Then keep its own frame clock. At each frame zero, send a 40-byte beacon with `radio_tx_at`: version, network ID, 32-bit frame counter, padding. After TxDone, stay in RX until the spare window and log every packet.
- [ ] **3.3 Drone state machine.** States: ACQUIRE (continuous RX until a valid beacon) and TRACK. In TRACK, open RX TDMA\_BEACON\_RX\_LEAD\_US (5 ms) before the expected beacon with a TDMA\_BEACON\_RX\_TIMEOUT\_US (35 ms) timeout, stopped on header detection (the SX126x default, not on preamble). HeaderValid arrives about 20.7 ms after the beacon starts, so a shorter timeout misses every beacon. In HOLDOVER, widen the lead and timeout by the predicted drift. On a beacon, update frame zero (Phase 2 method). Then for each slot in the mask, preload a status packet and call `radio_tx_at` at the slot start. Return to standby until the next beacon window.
- [ ] **3.4 Dummy status packet**, padded to 30 bytes: node ID, slot number, echoed beacon frame counter, beacon RSSI, beacon SNR, the node's last frame-zero correction in µs, and its state.
- [ ] **3.5 Duty-cycle accounting.** Sixty one-minute buckets of transmitted airtime (TxDone minus SetTx). Refuse to transmit, and log it, if the rolling hour would exceed 10%. A `BENCH_CONDUCTED` compile flag may raise the limit for the full-load option, but only with a hard 10-minute run limit, after which the node stops transmitting.
- [ ] **3.6 Ground log.** One CSV line per received packet: `frame, slot, node, rx_offset_us, rssi, snr, crc_ok`. `rx_offset_us` is HeaderValid time minus (slot start + `T_RX_HDRVALID_OFFSET_US`), so a perfect packet logs about 0. Also log missing expected slots.
- [ ] **3.7 Analysis script.** Python: completeness per slot, histogram of `rx_offset_us`, CRC errors, missing slots by frame, and measured duty cycle per node.

### Exit criteria

| Measure | Target |
| --- | --- |
| Default test duration | 1 hour (1800 frames, 16 200 slot packets) |
| Slot packets received with good CRC | ≥ 99.5% |
| CRC errors coinciding with neighbouring slot activity | 0 |
| `rx_offset_us`, worst case | Within ±1 ms |
| Measured duty, ground node | About 3.9% (77 ms beacon per 2 s frame), never above 10% |
| Measured duty, each drone node | About 9.3%, never above 10% |
| Transmit windows (logic analyser) | No overlap between TX-window pins of different nodes; each window starts in its own slot and ends before the next. Band-edge spill is not checked without an SDR; record it as untested. |
| Full-load option (if run) | 10 minutes, all 20 slots, same timing and CRC targets |

## Phase 4: Holdover and robustness

Phase 4 makes the link fail safely: a drone node may keep its slots on its own clock for a limited time, but must never transmit outside them.

### Firmware additions

- [ ] **4.1 HOLDOVER and SILENT states.** Missing a beacon moves TRACK to HOLDOVER: keep transmitting on the predicted frame zero. After `TDMA_HOLDOVER_FRAMES` (30) consecutive misses, go SILENT: no transmissions, continuous RX, back to ACQUIRE behaviour.
- [ ] **4.2 Wider search.** After 3 consecutive misses, listen continuously outside the node's own TX slots, so a ground node that restarted with a new frame phase is found quickly.
- [ ] **4.3 Drift correction.** Estimate the local TIM2 clock's error (TIM2 runs from the STM32 clock, not the SX1262 TCXO) from successive beacon intervals (expected 2 000 000 µs), smoothed with a moving average over about 16 frames. Apply it to predicted frame zeros during HOLDOVER. Budget: at ±50 ppm per side (100 ppm relative), 30 frames (60 s) of holdover drifts up to 6 ms, inside the 18.3 ms slot guard.
- [ ] **4.4 Re-acquisition rule.** Leave ACQUIRE or SILENT only after two beacons with consistent timing (interval within ±200 µs of 2.0 s). Never transmit from a single beacon after a reset.
- [ ] **4.5 Beacon validity.** A CRC error or wrong network ID counts as a missed beacon. A frame counter that goes backwards means the ground node restarted on a new phase: stop transmitting at once and drop to ACQUIRE. This is the fallback; the listen-first boot in 3.2 normally keeps the old phase.
- [ ] **4.6 Boot safety.** The radio stays in RX or standby from power-on until the first TRACK state; nothing transmits during initialisation.

### Test matrix

| Test | Method | Pass |
| --- | --- | --- |
| Short gap | Ground skips 1, 5 and 15 beacons | Drones keep slots; log drift at each gap |
| Holdover limit | Ground skips 35 beacons | Each drone transmits through 30 misses, then nothing until beacons return |
| Drift correction | Repeat the 29-beacon gap with 4.3 on and off | Worst-case slot error with correction at least 5× smaller, or both under 100 µs (then record that 4.3 is not needed) |
| Re-acquisition | Restore beacons after SILENT | Back in TRACK and transmitting within 2 frames |
| Drone power cycle | Cut a drone node's power at random points in the frame, 20 times | No burst outside its slots; rejoins within 3 frames |
| Late join | Power a drone after the others have run 10 minutes | Joins within 3 frames; no collisions |
| Ground restart | Reboot node A mid-run | A rejoins the existing phase from drone slot packets; frame counter keeps rising; drones stay in HOLDOVER then TRACK; no collisions with the new beacon |
| Bad beacons | Inject CRC errors and wrong network IDs | Treated as misses; no false sync |
| Temperature | Warm a drone node gently to about 50 °C, then cool, while running | Drift estimate tracks it; holdover still within guard |
| Soak | 8 hours, default slot set | ≥ 99.5% slots received; no collisions; duty within limits |

### Exit criteria

- [ ] Zero transmissions outside a node's own slot windows across all tests (checked with the TX-window pins, debug pins and ground log).
- [ ] Holdover drift over 30 missed beacons stays well inside the \~18 ms slot guard; the measured figure is recorded.
- [ ] Re-acquisition within 2 frames of beacons returning.
- [ ] 8-hour soak passes.
- [ ] Measured offsets, drift figures and test logs saved in the repo under `test-results/`.

## Working with Claude Code

Put the text below in `CLAUDE.md` at the repo root, export this doc as Markdown into `docs/phases-2-4.md`, and run one Claude Code session per task group.

```markdown
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

## Build and flash
- Use the mLRS build process for target rx-matek-mr900-30-g431kb (see mLRS docs).
- Flash by DFU. Role and node ID are compile-time defines: TDMA_ROLE, TDMA_NODE_ID.
```

### Suggested session order

1. Read the mLRS target files and report the DIO1 pin, free timers, the TX2 pin and interrupt priorities. No code yet.
2. Prerequisites from the scope section, if not already done.
3. Tasks 2.1–2.4, then 2.5–2.6 once you have bench measurements to give it.
4. Tasks 3.1–3.5, then 3.6–3.7 (logging and Python analysis).
5. Tasks 4.1–4.6, then a test-harness script for the Phase 4 matrix.

Give Claude Code the logic-analyser CSVs and ground logs from each test so it can update constants and check exit criteria.

## Open questions

Answered in session 1 from upstream mLRS (`hal-matek-mr-g431kb-common.h`, `rx-hal-matek-mr900-30-g431kb.h`, `CommonRx/mlrs-rx.cpp`, `hal-power-pa.h`, target `.ioc`):

- [x] **DIO1:** SX1262 DIO1 is PA15. mLRS reads it through EXTI15\_10, rising edge. PA15 alternate function 1 is TIM2\_CH1 per the STM32G431 datasheet (confirm in its AF table), so hardware input capture on TIM2 is possible.
- [x] **TX2 and SWD:** TX2 is PB3, the UART2 TX (SBus/out) pin in mLRS, free in the TDMA firmware. PB3 is also TIM2\_CH2, so debug pulses can come from TIM2 output compare. SWD pads PA13 (SWDIO) and PA14 (SWCLK) are broken out.
- [x] **TIM2:** used by mLRS as the receiver link clock (10 µs ticks, CC1 and CC3). It goes with the link logic and is the G431's only 32-bit timer, so the TDMA timebase takes it. TIM3 is the mLRS micros() counter; keep it. TIM1, TIM4, TIM6, TIM7, TIM8 and TIM15–TIM17 are unused.
- [x] **Interrupt priorities** (lower = higher): SWUART timer 9, link clock 10, serial and debug UARTs 11, out UART 12, DIO1 EXTI 13, FDCAN 14. With TIM2 input capture the RX timestamp is taken in hardware; TIM2 still needs the highest priority for scheduled TX (needs approval).
- [x] **Debug UART:** mLRS debug is LPUART1 TX on LT1 (PA2); TX1/RX1 (PA9/PA10) is the serial port. LR1 (PA3) is free for the TX-window pin.
- [x] **868 MHz power steps:** 20 dBm = SX −6, 24 dBm = −2, 27 dBm = 2, 30 dBm = 10. These are mLRS nominal outputs; the cap clamps requests to 24 dBm before this lookup.
- [x] **MCU clock:** 8 MHz HSE crystal, PLL to 170 MHz, timer clock 170 MHz, so a divide-by-170 gives exactly 1 MHz.

Still open:

- [ ] Crystal ppm rating for the 8 MHz HSE; task 2.1 measures it.
- [ ] Which TCXO mode the SX126x driver selects for this target (driver default is 1.8 V on DIO3).
- [ ] Measured output at the 24 dBm setting; needs a power meter or spectrum analyser.
- [ ] Does the bench supply hold 5 V for four nodes transmitting in sequence?
- [ ] Band-edge emissions and actual output power are unverified without an SDR or spectrum analyser. Borrow one, or add a low-cost RF log-detector board on a spare attenuator tap, before any over-the-air test.
- [ ] USB interrupt priority: the mLRS USB device code runs its interrupt at priority 0 (highest, set in usbd\_conf.h). Before Phase 2 scheduled transmit, either lower it below TIM2 or confirm on the logic analyser that SetTx jitter stays under 20 µs with USB logging active (needs approval to change).
