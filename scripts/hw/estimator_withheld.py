#!/usr/bin/env python3
"""E6: the estimator against a camera it never saw (`fabric_run.sh estimator-withheld <run> --withhold blue|overhead`).

The estimator uses the overhead and Blue's wrist camera (cell/estimator.yaml observation_topics; Silver's view awaits
its hand-eye check). For a recorded run this replays the estimator offline (fabric_state_estimator.replay, no ROS graph)
twice over the same window, from the bag's raw perception factors (fabric_reference_reconstruction.bag: camera
observations, per-view gloves, capture-time TF, robot grips):
  all       every camera the run's estimator used (in-sample reference, same replay code)
  withheld  the same without one camera
and scores both against that camera's depth (twin.depth.npz) exactly as `fabric_run.sh audit` scores the live estimate
(8 x 8 estimate upsampled to the twin mesh, signed point-to-sheet distance). Coverage: fraction of residuals within
1.96 sd of the estimate's own normal variance plus depth noise (want ~95 %). Writes <run>/estimator_withheld_<cam>.json.
GPU (Newton ensemble); minutes per window.
"""
import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import zoom

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from identify_replay import data_file  # noqa: E402
from twin_audit import signed, stats, surface  # noqa: E402

CAMERAS = {'blue': '/oarbot_blue/azure_kinect/fabric/observations', 'overhead': '/nuc/fabric/observations'}
DEPTH_STD_M = .01   # depth noise added to the estimate's variance for coverage (Azure Kinect at 1-2 m, ~1 cm)


def replay_run(bag, config_path, window_ns, stride):
    from fabric_reference_reconstruction.bag import extract, read_bag
    from fabric_state_estimator.config import load_config
    from fabric_state_estimator.observations import ObservationBuilder
    from fabric_state_estimator.physics import make_physics
    from fabric_state_estimator.replay import replay
    mesh, physics, observer_spec, runtime, options, _ = load_config(config_path)
    lo, hi = window_ns

    def messages():
        for topic, message in read_bag(bag, 'mcap'):
            stamp = getattr(getattr(message, 'header', None), 'stamp', None)
            if topic in ('/tf', '/tf_static') or stamp is None or lo <= stamp.sec*10**9+stamp.nanosec <= hi:
                yield topic, message
    frames, dropped = extract(messages(), mesh, runtime, max_frames=10**7, stride=stride)
    states, snapshots = replay(mesh, make_physics(mesh, physics), frames, observer_spec, ObservationBuilder(mesh, **options))
    return states, len(frames), len(dropped), sum(bool(s['degraded']) for s in snapshots)


def score(states, depth, cam_index, n):
    offsets = np.r_[0, np.cumsum(depth['counts'])]
    e_ns, e_q, e_cov = states['times_ns'], states['positions'], states['covariance']
    m = int(round(np.sqrt(e_q.shape[1])))
    rs, zs = [], []
    for j in np.flatnonzero(depth['camera'] == cam_index):
        k = int(np.argmin(abs(e_ns-depth['stamps_ns'][j])))
        if abs(e_ns[k]-depth['stamps_ns'][j]) > .05e9:
            continue
        q = e_q[k].reshape(m, m, 3)
        fine = np.stack([zoom(q[..., c], (n-1)/(m-1)+1e-9, order=1) for c in range(3)], -1)
        flat, normals = surface(fine)
        r, index, _ = signed(flat, normals, depth['points'][offsets[j]:offsets[j+1]].astype(float))
        rs.append(r)
        # Coverage: the nearest coarse node's position covariance along the local normal.
        f = fine.shape[0]                      # zoom rounds the grid up (32 x 32 from 8 x 8), as in the audit
        fine_rc = np.stack(np.unravel_index(index, (f, f)), 1).astype(float)*(m-1)/(f-1)
        node = np.round(fine_rc).astype(int)
        node_index = node[:, 0]*m+node[:, 1]
        cov = e_cov[k][node_index, :3, :3]
        var = np.einsum('ij,ijk,ik->i', normals[index], cov, normals[index])+DEPTH_STD_M**2
        zs.append(r/np.sqrt(np.maximum(var, 1e-12)))
    if not rs:
        return None
    r, z = np.concatenate(rs), np.concatenate(zs)
    out = stats(r)
    ok = np.isfinite(z)
    out['coverage_95'] = round(float(np.mean(np.abs(z[ok]) < 1.96)), 3) if ok.any() else None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--withhold', choices=sorted(CAMERAS), required=True)
    ap.add_argument('--window', default='', help='START:END seconds from the first depth frame (default: whole run)')
    ap.add_argument('--stride', type=int, default=1, help='use every n-th synchronized camera frame')
    a = ap.parse_args()
    run = a.run.resolve()
    est_cfg = yaml.safe_load((run / 'cell' / 'estimator.yaml').read_text())
    topics = est_cfg['runtime']['observation_topics']
    withheld = CAMERAS[a.withhold]
    if withheld not in topics or len(topics) < 2:
        sys.exit(f'{a.withhold} is not one of at least two estimator cameras in this run: {topics}')
    cam_index = topics.index(withheld)       # twin.depth.npz camera index = position in observation_topics
    depth = dict(np.load(data_file(run, 'twin.depth.npz')))
    n = int(round(np.sqrt(len(np.load(data_file(run, 'twin.npz'))['uv']))))
    t0 = int(depth['stamps_ns'][0])
    if a.window:
        s, e = (float(x) for x in a.window.split(':'))
        window = (t0+int(s*1e9), t0+int(e*1e9))
    else:
        window = (t0, int(depth['stamps_ns'][-1]))
    bag = data_file(run, 'bag')
    result = dict(run=str(run), withheld=a.withhold, withheld_topic=withheld, camera_index=cam_index,
                  window_s=[(window[0]-t0)/1e9, (window[1]-t0)/1e9], stride=a.stride)
    with tempfile.TemporaryDirectory() as tmp:
        for name, used in (('all', topics), ('withheld', [t for t in topics if t != withheld])):
            cfg = dict(est_cfg, runtime=dict(est_cfg['runtime'], observation_topics=used))
            path = Path(tmp) / f'estimator_{name}.yaml'
            path.write_text(yaml.safe_dump(cfg, sort_keys=False))
            begun = time.perf_counter()
            states, frames, dropped, degraded = replay_run(bag, path, window, a.stride)
            result[name] = dict(cameras=used, frames=frames, dropped=dropped, degraded=degraded,
                                seconds=round(time.perf_counter()-begun, 1),
                                vs_withheld_camera=score(states, depth, cam_index, n))
            print(f'{name:9s} cameras {len(used)}: {frames} frames ({degraded} degraded), {result[name]["seconds"]:.0f} s; '
                  f'vs {a.withhold} depth: {result[name]["vs_withheld_camera"]}', flush=True)
    out = run / f'estimator_withheld_{a.withhold}.json'
    out.write_text(json.dumps(result, indent=1))
    print(f'written {out}')


if __name__ == '__main__':
    main()
