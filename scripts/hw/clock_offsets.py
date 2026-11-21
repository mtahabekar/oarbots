#!/usr/bin/env python3
"""Clock offset of lab machines against this laptop (gate H9), over SSH, no tools needed there.

For each host: eight timed round trips that read the remote clock (`date +%s.%N`) on one SSH
connection; the offset is the remote clock minus the midpoint of the local send and receive times,
median of the three shortest round trips (wired: about 1 ms, so the uncertainty is about 0.5 ms).
Also prints the remote time zone, its NTP-synchronized flag and which time daemon runs.

    python3 scripts/hw/clock_offsets.py oarbot_blue@192.168.1.71 oarbot_silver@192.168.1.110 nuc@192.168.1.90
Set LAB_SSH to replace the ssh command (as for lab.sh).
"""
import os
import statistics
import subprocess
import sys
import time

REMOTE = ('for i in 1 2 3 4 5 6 7 8; do read x; date +%s.%N; done; '
          '(timedatectl show -p Timezone -p NTPSynchronized --value; systemctl is-active chrony systemd-timesyncd; '
          'timedatectl show-timesync -p ServerAddress --value || echo none) 2>/dev/null | tr "\\n" " "')


def offset(host):
    cmd = os.environ.get('LAB_SSH', 'ssh').split() + [host, REMOTE]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    samples = []
    for _ in range(8):
        t0 = time.time()
        p.stdin.write('x\n')
        p.stdin.flush()
        remote = float(p.stdout.readline())
        t1 = time.time()
        samples.append((t1 - t0, remote - (t0 + t1) / 2))
    rest = p.stdout.read().split()
    p.wait()
    best = sorted(samples)[:3]
    return statistics.median(o for _, o in best), best[0][0], rest


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for host in sys.argv[1:]:
        try:
            off, rtt, rest = offset(host)
            tz, synced, *daemons = rest + ['?'] * 5
            # server: the NTP server timesyncd follows (the lab's is the laptop, 192.168.1.200; a host that booted
            # without the lab link falls back to campus NTP: `fabric_run.sh clocks --fix`)
            print(f'{host:30s} {off * 1e3:+8.1f} ms  (round trip {rtt * 1e3:.1f} ms)  tz {tz}  synchronized {synced}  '
                  f'chrony {daemons[0]}  timesyncd {daemons[1]}  server {daemons[2] or "none"}')
        except (ValueError, OSError) as e:
            print(f'{host:30s} failed: {e}')


if __name__ == '__main__':
    main()
