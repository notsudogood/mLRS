# Prerequisite link test (phases 0–1)

Firmware: `mLRS/tdma/tdma-main.cpp`, built with `tools/run_make_tdma.py`.

Two units exchange fixed 30-byte packets at 869.525 MHz, SF8, 250 kHz, CR 4/5, explicit header, CRC on, 17 dBm by default.
The ground node sends a ping once per second and waits up to 200 ms for a pong.
The drone node listens continuously and answers each ping.
Each packet is ~61.7 ms on air, so each node runs at ~6.2% duty.

Use one ground node and one drone node only. Drones answer at once, so two drones would collide.

## Build

```bash
python3 tools/run_make_tdma.py --role ground
```

```bash
python3 tools/run_make_tdma.py --role drone --node 1
```

## Procedure

1. Fit the attenuator chain (or antenna) to both units before powering them. Never transmit into an open port.
2. Flash `tdma-ground-n0-*.hex` to unit A and `tdma-drone-n1-*.hex` to unit B by DFU.
3. Connect a USB–UART adapter to TX1/RX1 and GND of each unit, 115200 8N1, and log each to a file.
4. Power both units. Both print the radio settings line; red LED off once the radio is configured.
5. Run for 10 minutes (600 pings).
6. Pass: ≥ 99% of pings answered (`pong` lines on A), RSSI about −53 dBm at 17 dBm through ~70 dB loss, SNR positive, no `tx_fail`.

LEDs: green toggles per good packet; red on = no link (drone: no ping for 3 s; ground: last pong timed out); fast red blink = SX1262 not responding.

Optional: the logic analyser on TX2 (debug pulses: 20 µs HeaderValid, 40 µs SetTx, 80 µs TxDone) and LR1 (high while transmitting) already works with this firmware.

## CSV log format

Lines starting with `#` are comments (startup info, column names).

Ground (A):

| Column | Meaning |
| --- | --- |
| event | `pong`, `timeout`, `crc_err`, `bad_pkt`, `tx_fail` |
| seq | ping sequence number |
| node | drone node ID that answered |
| rssi, snr | pong as received at A (dBm, dB) |
| remote_rssi, remote_snr | ping as received at the drone (dBm, dB) |
| crc_ok | 1 or 0 |

Drone (B):

| Column | Meaning |
| --- | --- |
| event | `ping`, `crc_err`, `bad_pkt`, `skip_duty`, `tx_fail` |
| seq | ping sequence number |
| node | sender node ID (0 = ground) |
| rssi, snr | ping as received at B (dBm, dB) |
| crc_ok | 1 or 0 |
