"""Write a scripted human (leader) motion that `leader_replay` can play into the plant.

    ros2 run fabric_sim_cell leader_script OUT.npz --start 12 --move 0 0 -0.45 0.08 --hold 2 --move 0.1 0 0 0.05 ...

Both hands move together, as a person carrying the sheet does: each `--move DX DY DZ SPEED`
translates both leader corners by (DX, DY, DZ) metres at SPEED m/s with a smooth start and
stop, each `--hold S` waits. The output has the same `times_ns` / `corners` arrays as the plant's
truth file; only the two leader rows are used.
"""
import argparse
import sys

import numpy as np


def build(segments, start, total, rate=60., origin=((-.42, .42, 1.05), (.42, .42, 1.05)), gather=0., gather_start=10.5):
    times = np.arange(1, int(round(total*rate))+1)/rate
    offset = np.zeros((len(times), 3))
    # `gather`: each hand first moves this far toward the other (a person does not hold a sheet
    # stretched exactly to its width), over 2 s from `gather_start`.
    inward = .5*(1-np.cos(np.pi*np.clip((times-gather_start)/2., 0., 1.)))*gather
    t0, base = start, np.zeros(3)
    for kind, values in segments:
        if kind == 'hold':
            t0 += values[0]
            continue
        delta, speed = np.array(values[:3]), values[3]
        duration = max(np.linalg.norm(delta)/speed*1.5, 1e-6)  # cosine profile: peak speed = pi/2 x mean
        phase = np.clip((times-t0)/duration, 0., 1.)
        offset += np.outer(.5*(1-np.cos(np.pi*phase)), delta)
        t0 += duration; base += delta
    corners = np.zeros((len(times), 4, 3))
    corners[:, 2] = np.asarray(origin[0])+offset
    corners[:, 3] = np.asarray(origin[1])+offset
    corners[:, 2, 0] += inward
    corners[:, 3, 0] -= inward
    return (times*1e9).round().astype(np.int64), corners, t0


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('output')
    parser.add_argument('--start', type=float, default=12., help='Simulation time of the first segment (s)')
    parser.add_argument('--total', type=float, default=60.)
    parser.add_argument('--gather', type=float, default=0., help='Each hand first moves this far toward the other (m)')
    options, rest = parser.parse_known_args(args)
    segments, i = [], 0
    while i < len(rest):
        if rest[i] == '--move':
            segments.append(('move', [float(v) for v in rest[i+1:i+5]])); i += 5
        elif rest[i] == '--hold':
            segments.append(('hold', [float(rest[i+1])])); i += 2
        else:
            parser.error(f'unknown argument {rest[i]}')
    times, corners, end = build(segments, options.start, options.total, gather=options.gather)
    if end > options.total:
        sys.exit(f'the script needs {end:.1f} s, more than --total {options.total}')
    np.savez(options.output, times_ns=times, corners=corners)
    print(f'{len(segments)} segments, motion ends at {end:.1f} s -> {options.output}')


if __name__ == '__main__':
    main()
