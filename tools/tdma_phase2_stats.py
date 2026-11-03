#!/usr/bin/env python3
'''
Swarm C2 TDMA - Phase 2 timing test statistics

usage:
  python3 tools/tdma_phase2_stats.py ground.csv drone.csv [drone2.csv ...]

Reads the CSV logs of the Phase 2 timing test firmware (mLRS/tdma/tdma-main.cpp).
Lines may carry a leading wall-clock prefix (e.g. from `ts`); comment lines (#) are skipped.

Ground (event tx):
  settx_err_us            SetTx issue time minus its scheduled time: interrupt latency/jitter
  txdone_minus_settx_us   SetTx to TxDone: TX latency + airtime (~61.7 ms expected)
Drone (event rx):
  err_us                  HeaderValid interval minus the expected interval (n x 1 000 000 us)
  ppm_x10                 drone clock error relative to the ground, 0.1 ppm
The jitter of err_us is the combined jitter of ground SetTx timing, TX start
and HeaderValid detection on the drone; slow drift in it is the clock error.
'''
import sys
import statistics


def rows(path, event):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or '#' in line.split(',')[0]:
                continue
            # drop an optional "HH:MM:SS " prefix
            if ' ' in line.split(',')[0]:
                line = line.split(' ', 1)[1]
            cols = line.split(',')
            if cols[0] == event:
                out.append(cols)
    return out


def ints(rows_, idx):
    vals = []
    for r in rows_:
        if idx < len(r) and r[idx] != '':
            try:
                vals.append(int(r[idx]))
            except ValueError:
                pass
    return vals


def summary(name, vals, unit):
    if not vals:
        print('  %-28s no data' % name)
        return
    sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    print('  %-28s n=%-5d mean=%9.1f  sd=%7.1f  min=%7d  max=%7d %s' % (
        name, len(vals), statistics.mean(vals), sd, min(vals), max(vals), unit))


def count(path, event):
    return len(rows(path, event))


def ground(path):
    print('ground:', path)
    tx = rows(path, 'tx')
    print('  tx=%d late=%d tx_fail=%d' % (len(tx), count(path, 'late'), count(path, 'tx_fail')))
    summary('settx_err_us', ints(tx, 5), 'us')
    summary('txdone_minus_settx_us', ints(tx, 6), 'us')
    seqs = ints(tx, 1)
    if seqs:
        missing = (max(seqs) - min(seqs) + 1) - len(set(seqs))
        print('  seq %d..%d, %d periods without a tx line' % (min(seqs), max(seqs), missing))


def drone(path, ground_seqs):
    print('drone:', path)
    rx = rows(path, 'rx')
    print('  rx=%d crc_err=%d bad_pkt=%d' % (len(rx), count(path, 'crc_err'), count(path, 'bad_pkt')))
    seqs = set(ints(rx, 1))
    if ground_seqs:
        common = [s for s in ground_seqs if min(seqs, default=0) <= s <= max(seqs, default=0)]
        if common:
            got = len([s for s in common if s in seqs])
            print('  received %d of %d packets sent while it was listening (%.2f%%)' % (
                got, len(common), 100.0 * got / len(common)))
    err = ints(rx, 4)
    summary('err_us (interval jitter)', err, 'us')
    ppm = [v / 10.0 for v in ints(rx, 5)]
    if ppm:
        print('  %-28s mean=%+.2f ppm  sd=%.2f ppm  (drone clock vs ground)' % ('clock error', statistics.mean(ppm), statistics.pstdev(ppm)))
        # slow drift: mean over the first and last tenth
        k = max(1, len(ppm) // 10)
        print('  %-28s first %d: %+.2f ppm, last %d: %+.2f ppm' % ('clock error drift', k, statistics.mean(ppm[:k]), k, statistics.mean(ppm[-k:])))
    summary('rssi', ints(rx, 6), 'dBm')
    summary('snr', ints(rx, 7), 'dB')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    ground_seqs = []
    for p in sys.argv[1:]:
        if rows(p, 'tx') or rows(p, 'late'):
            ground(p)
            ground_seqs = ints(rows(p, 'tx'), 1)
    for p in sys.argv[1:]:
        if rows(p, 'rx') or rows(p, 'crc_err'):
            drone(p, ground_seqs)
