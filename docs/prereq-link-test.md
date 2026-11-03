# Prerequisite link test (phases 0–1)

Firmware: `mLRS/tdma/tdma-main.cpp`, built with `tools/run_make_tdma.py`.

Two units, both plugged into a Linux laptop by USB-C, exchange fixed 30-byte packets at 869.525 MHz, SF8, 250 kHz, CR 4/5, explicit header, CRC on, 17 dBm by default.
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
2. Flash `tdma-ground-n0-*` to unit A and `tdma-drone-n1-*` to unit B by DFU: hold the button while plugging in USB-C, then either STM32CubeProgrammer (USB, erase + program the .hex) or dfu-util with the .bin:

   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D tools/build/firmware/tdma-ground-n0-<hash>.bin
   ```
3. Plug both units into the laptop by USB-C. USB powers them and carries the log; each appears as `/dev/ttyACM*`, with a stable name under `/dev/serial/by-id/` (serial number = chip ID). The same log is also on TX1 at 115200 8N1 if you prefer a USB–UART adapter.
4. Start logging each port straight after plugging in (the firmware waits 1.5 s for enumeration before its header lines; lines are dropped while no program has the port open):

   ```bash
   ls -l /dev/serial/by-id/
   ```

   ```bash
   stty -F /dev/ttyACM0 raw -echo && cat /dev/ttyACM0 | tee ground.csv
   ```

   Ground lines start with `pong`/`timeout`, drone lines with `ping`. Your user needs to be in the `dialout` group. If ModemManager is running it may probe the ports for a few seconds after plug-in; stop it (`sudo systemctl stop ModemManager`) if lines go missing.
5. Red LED goes off once the radio is configured.
6. Run for 10 minutes (600 pings).
7. Pass: ≥ 99% of pings answered (`pong` lines on A), RSSI about −53 dBm at 17 dBm through ~70 dB loss, SNR positive, no `tx_fail`.

LEDs: green toggles per good packet; red on = no link (drone: no ping for 3 s; ground: last pong timed out); fast red blink = SX1262 not responding.

Optional: the logic analyser on TX2 (debug pulses: 20 µs HeaderValid, 40 µs SetTx, 80 µs TxDone) and LR1 (high while transmitting) already works with this firmware.

## CSV log format

Lines starting with `#` are comments (startup info, column names).

Every 10 s each node also prints a status line, so a logger opened late still identifies the node:

```
# alive,uptime_s=120,role=drone,node=1,power_dbm=17,tx=0,rx_ok=0,rx_err=0
```

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
