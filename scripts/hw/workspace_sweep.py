#!/usr/bin/env python3
"""Where each arm's grip can go around the grasp pose while it holds the sheet: a MoveIt sweep, and the box.

Read-only: sends nothing to the arms. The fabric bridge moves the grip by translation only, holding the tool
orientation it had at arming, through joint velocities (differential IK). So a grip position is usable when
  - MoveIt finds a collision-free IK solution with the grasp orientation (the robot's own model and scene),
  - that solution is on the same branch as its neighbours (the arm gets there by a continuous motion:
    no joint jumps more than --max-step-deg between grid neighbours), and
  - the arm's wrist links stay --clearance from the other arm's, with the other arm anywhere in its box.

Step 1 (both sessions running, laptop session environment; about 3-5 min per robot, run both at once):
    python3 scripts/hw/workspace_sweep.py sweep --robot blue   --out <dir>
    python3 scripts/hw/workspace_sweep.py sweep --robot silver --out <dir>
Step 2 (offline): the largest box around each grasp grip where every grid point inside, and one grid step
beyond each face (the overshoot allowance), is usable, and the pair of boxes keeps the arms apart:
    python3 scripts/hw/workspace_sweep.py box --out <dir>
"""
import argparse
import itertools
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'oarbot_runtime'))  # the bridge's own kinematics
LINKS = ('j2n6s300_end_effector', 'j2n6s300_link_5', 'j2n6s300_link_4', 'camera_base')


def sweep(args):
    import rclpy
    import yaml
    from grasp_target import CELL, Target, base_poses
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from oarbot_runtime.arm_kinematics import Chain, joint_rates, rotation_vector
    from urdf_parser_py.urdf import URDF
    grasp = np.array(yaml.safe_load((CELL / 'grasp.yaml').read_text())[args.robot])
    W = base_poses()[args.robot]
    rclpy.init()
    node = Target(args.robot)
    urdf = []
    node.create_subscription(String, node.ns + '/robot_description', lambda m: urdf.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.wait(lambda: node.joints is not None and urdf, 'joint states and robot_description')
    prefix = f'oarbot_{args.robot}/'
    robot = URDF.from_xml_string(urdf[0])
    chain = Chain(robot, prefix + 'j2n6s300_link_base', prefix + 'j2n6s300_end_effector')
    limits = []
    for name in chain.names:
        j = robot.joint_map[name]
        limits.append((j.limit.lower, j.limit.upper) if j.type == 'revolute' and j.limit is not None else (-np.inf, np.inf))
    limits = np.array(limits)
    # the arm base (chain frame) in the world: base_link pose x the fixed base_link -> link_base
    Wl = W @ node.links(list(grasp), ['j2n6s300_link_base'])['j2n6s300_link_base']
    R = Wl[:3, :3]
    T0 = chain.fk(grasp)
    grip0 = (Wl @ T0)[:3, 3]
    cap = math.radians(args.cap_dps)

    def solve(q, target):
        """Newton steps with the bridge's damped least squares; position to 0.5 mm, orientation to 0.5 deg."""
        q = q.copy()
        for _ in range(60):
            J, T = chain.jacobian(q)
            e = np.r_[target - T[:3, 3], rotation_vector(T0[:3, :3] @ T[:3, :3].T)]
            if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < math.radians(.5):
                return q
            q = q + joint_rates(J, e, 0.01)
        return None

    def speed(q):
        """Grip speed (m/s) the arm can deliver in every direction, orientation held, no joint over the cap."""
        J, _ = chain.jacobian(q)
        A = np.linalg.pinv(J)[:, :3]
        return float(cap / np.max(np.linalg.norm(A, axis=1)))

    xs, ys, zs = (np.round(np.arange(lo, hi + 1e-9, args.step), 4) for lo, hi in
                  ((args.x[0], args.x[1]), (args.y[0], args.y[1]), (args.z[0], args.z[1])))
    shape = (len(xs), len(ys), len(zs))
    start = tuple(int(np.argmin(np.abs(a))) for a in (xs, ys, zs))
    joints = np.full(shape + (6,), np.nan)
    links = np.full(shape + (len(LINKS), 3), np.nan)
    speeds = np.full(shape, np.nan)
    status = np.zeros(shape, np.int8)       # 0 not reached, 1 usable, 2 out of reach, 3 joint limit, 4 collision
    joints[start], status[start], speeds[start] = grasp, 1, speed(grasp)
    queue, t0 = deque([start]), time.monotonic()
    while queue:
        cur = queue.popleft()
        for d in itertools.product((-1, 0, 1), repeat=3):
            if sum(map(abs, d)) != 1:
                continue
            nb = tuple(c + e for c, e in zip(cur, d))
            if not all(0 <= n < s for n, s in zip(nb, shape)) or status[nb] != 0:
                continue
            target = T0[:3, 3] + R.T @ np.array([xs[nb[0]], ys[nb[1]], zs[nb[2]]])
            q = solve(joints[cur], target)
            if q is None:
                status[nb] = 2
                continue
            if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
                status[nb] = 3
                continue
            ok, _ = node.contacts(list(q))
            if not ok:
                status[nb] = 4
                continue
            joints[nb], status[nb], speeds[nb] = q, 1, speed(q)
            pts = node.links(list(q), list(LINKS))
            links[nb] = [(W @ pts[n])[:3, 3] for n in LINKS]
            queue.append(nb)
    pts = node.links(list(grasp), list(LINKS))
    links[start] = [(W @ pts[n])[:3, 3] for n in LINKS]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / f'{args.robot}.npz', xs=xs, ys=ys, zs=zs, grip0=grip0, grasp=grasp, joints=joints, links=links,
             status=status, speeds=speeds, step=args.step, cap_dps=args.cap_dps)
    counts = {k: int((status == v).sum()) for k, v in (('usable', 1), ('out_of_reach', 2), ('joint_limit', 3), ('collision', 4), ('not_reached', 0))}
    print(f'{args.robot}: grasp grip {np.round(grip0, 3).tolist()}, speed there {speeds[start]*100:.0f} cm/s at {args.cap_dps:.0f} deg/s; '
          f'{counts}, {time.monotonic() - t0:.0f} s')
    node.destroy_node()
    rclpy.shutdown()


def boxes(usable):
    """Every box (index ranges, inclusive) that contains the grasp cell and whose cells, extended by one
    cell beyond every face, are all usable. Yields (lo, hi) with lo, hi the box's own faces."""
    ok = usable.astype(np.int32)
    S = np.zeros(tuple(n + 1 for n in ok.shape), np.int32)
    S[1:, 1:, 1:] = ok.cumsum(0).cumsum(1).cumsum(2)

    def full(lo, hi):   # inclusive
        a, b = lo, [h + 1 for h in hi]
        s = (S[b[0], b[1], b[2]] - S[a[0], b[1], b[2]] - S[b[0], a[1], b[2]] - S[b[0], b[1], a[2]]
             + S[a[0], a[1], b[2]] + S[a[0], b[1], a[2]] + S[b[0], a[1], a[2]] - S[a[0], a[1], a[2]])
        return s == np.prod(b - np.array(a))
    return full


def box(args):
    out = Path(args.out)
    data = {r: dict(np.load(out / f'{r}.npz')) for r in ('blue', 'silver')}
    result = {}
    candidates = {}
    for r, d in data.items():
        usable = (d['status'] == 1) & (np.nan_to_num(d['speeds']) >= args.min_speed)
        full = boxes(usable)
        c = tuple(int(np.argmin(np.abs(a))) for a in (d['xs'], d['ys'], d['zs']))
        n = usable.shape
        # Balanced growth: extend the six faces in turn, one grid step at a time, while every point inside and
        # one step beyond each face stays usable. (Maximising volume instead trades one direction away: it gave
        # 33 cm down and 0 up.) The inner x face is capped in turn for the clearance between the arms.
        candidates[r] = []
        inner_axis_face = 1 if r == 'blue' else 0       # blue's inner face is +x, silver's -x
        for cap in range(n[0]):
            lo, hi = list(c), list(c)
            grown = True
            while grown:
                grown = False
                for ax in range(3):
                    for side in (0, 1):
                        trial_lo, trial_hi = lo.copy(), hi.copy()
                        if side == 0:
                            trial_lo[ax] -= 1
                        else:
                            trial_hi[ax] += 1
                        if ax == 0 and side == inner_axis_face and abs((trial_hi if side else trial_lo)[0] - c[0]) > cap:
                            continue
                        if trial_lo[ax] < 1 or trial_hi[ax] > n[ax] - 2:
                            continue
                        if full([v - 1 for v in trial_lo], [v + 1 for v in trial_hi]):
                            lo, hi, grown = trial_lo, trial_hi, True
            box_ = (tuple(lo), tuple(hi))
            if box_ not in candidates[r]:
                candidates[r].append(box_)
        print(f'{r}: usable {int(usable.sum())} of {usable.size} grid points (speed >= {args.min_speed*100:.0f} cm/s); '
              f'{len(candidates[r])} candidates by inner-face cap')
    # pairwise clearance between the arms' wrist links, each anywhere in its box (including the margin cells)
    best = None
    for bb in candidates['blue']:
        for sb in candidates['silver']:
            lb = data['blue']['links'][bb[0][0] - 1:bb[1][0] + 2, bb[0][1] - 1:bb[1][1] + 2, bb[0][2] - 1:bb[1][2] + 2].reshape(-1, 3)
            ls = data['silver']['links'][sb[0][0] - 1:sb[1][0] + 2, sb[0][1] - 1:sb[1][1] + 2, sb[0][2] - 1:sb[1][2] + 2].reshape(-1, 3)
            # the closest pair is at the inner x faces: check the 400 innermost link points of each arm
            lb = lb[np.argsort(-lb[:, 0])[:400]]
            ls = ls[np.argsort(ls[:, 0])[:400]]
            dmin = float(np.min(np.linalg.norm(lb[:, None] - ls[None], axis=2)))
            if dmin < args.clearance:
                continue
            # the smallest reach from the grasp through any face (both arms), then the volume
            cb = tuple(int(np.argmin(np.abs(a))) for a in (data['blue']['xs'], data['blue']['ys'], data['blue']['zs']))
            cs = tuple(int(np.argmin(np.abs(a))) for a in (data['silver']['xs'], data['silver']['ys'], data['silver']['zs']))
            reach = min(min(np.array(c_) - np.array(b_[0])) for c_, b_ in ((cb, bb), (cs, sb)))
            reach = min(reach, min(min(np.array(b_[1]) - np.array(c_)) for c_, b_ in ((cb, bb), (cs, sb))))
            v = (reach, np.prod(np.array(bb[1]) - np.array(bb[0])) * np.prod(np.array(sb[1]) - np.array(sb[0])))
            print(f'  candidate blue {bb} silver {sb}: closest face {reach} steps, arms {dmin*100:.0f} cm apart')
            if best is None or v > best[0]:
                best = (v, bb, sb, dmin)
    if best is None:
        raise SystemExit(f'No pair of boxes keeps the arms {args.clearance} m apart')
    _, bb, sb, dmin = best
    for r, b in (('blue', bb), ('silver', sb)):
        d = data[r]
        lo = np.array([d['xs'][b[0][0]], d['ys'][b[0][1]], d['zs'][b[0][2]]])
        hi = np.array([d['xs'][b[1][0]], d['ys'][b[1][1]], d['zs'][b[1][2]]])
        sl = d['joints'][b[0][0]:b[1][0] + 1, b[0][1]:b[1][1] + 1, b[0][2]:b[1][2] + 1].reshape(-1, 6)
        change = np.degrees(np.nanmax(np.abs(sl - d['grasp']), axis=0))
        result[r] = dict(grasp_grip=np.round(d['grip0'], 3).tolist(), offset_min=lo.round(3).tolist(), offset_max=hi.round(3).tolist(),
                         workspace_min=(d['grip0'] + lo).round(3).tolist(), workspace_max=(d['grip0'] + hi).round(3).tolist(),
                         max_joint_change_deg=np.round(change).astype(int).tolist())
        print(f'{r}: box around the grasp grip x {lo[0]*100:+.0f}/{hi[0]*100:+.0f} cm, y {lo[1]*100:+.0f}/{hi[1]*100:+.0f} cm, '
              f'z {lo[2]*100:+.0f}/{hi[2]*100:+.0f} cm (one usable step of {float(d["step"])*100:.0f} cm beyond every face)')
        print(f'   workspace_min: {result[r]["workspace_min"]}\n   workspace_max: {result[r]["workspace_max"]}')
        print(f'   largest joint change from the grasp inside the box: {result[r]["max_joint_change_deg"]} deg')
    print(f'closest wrist links between the arms, both anywhere in their boxes: {dmin*100:.0f} cm (required {args.clearance*100:.0f})')
    result['closest_between_arms_m'] = round(dmin, 3)
    (out / 'boxes.json').write_text(json.dumps(result, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('sweep')
    s.add_argument('--robot', choices=('blue', 'silver'), required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--step', type=float, default=0.03)
    # x: along the robots' edge (each arm's inner side toward the other), y: toward the person, z: up
    s.add_argument('--x', type=float, nargs=2, default=(-0.24, 0.24))
    s.add_argument('--y', type=float, nargs=2, default=(-0.21, 0.36))
    s.add_argument('--z', type=float, nargs=2, default=(-0.30, 0.24))
    s.add_argument('--cap-dps', type=float, default=36., help='joint-rate cap for the speed map (deg/s)')
    b = sub.add_parser('box')
    b.add_argument('--out', required=True)
    b.add_argument('--clearance', type=float, default=0.15, help='wrist links of the two arms at least this far apart (m)')
    b.add_argument('--min-speed', type=float, default=0.08,
                   help='every point delivers at least this grip speed in every direction at the sweep cap (m/s)')
    args = ap.parse_args()
    sweep(args) if args.cmd == 'sweep' else box(args)


if __name__ == '__main__':
    main()
