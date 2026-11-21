#!/usr/bin/env python3
"""Write calibrated grip points into the cell configuration (`fabric_run.sh grip-apply <result.json> --bag <bag>`).

Reads the result of scripts/hw/grip_point_search.py and turns each corner's fitted grip point into the place
where that corner's measurement is defined, so every consumer (twin, estimator, stretch guard, bridges) uses it:

  robots (blue, silver)  world offset -> translation of <robot>/fabric_grip in its j2n6s300_end_effector frame
                         (experiments/hardware/cell/calibration.yaml). The end effector's orientation at the grasp
                         pose is read from the recorded TF in --bag.
  hands (left, right)    world offset -> fusion.yaml world_grip_offset_m [left xyz, right xyz]
  insets (all four)      twin.yaml sheet.grip_insets_m [blue, silver, right, left]: one held particle per corner

Robot and hand offsets are ADDED to the ones the recording was made with (its cell/ snapshot), since the search fits
corrections relative to the recorded grips; insets are absolute (the search's --config).

A world offset is down/in/side in the search's convention: down = -z; in = toward the opposite edge (robots toward
the hands, hands toward the robots), horizontal; side = along the robot edge toward Silver. The directions come
from the fitted recording's grips at the window start, as in the search.

Without --write it prints what it would change. `measured` in calibration.yaml stays false: these points are
identified from depth, not measured with the fixture (gate H15). Keep the printed numbers in the lab log.
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts/hw'))
CORNERS = ('blue', 'silver', 'right', 'left')


def quaternion_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def tool_rotations(bag, seconds):
    """World rotation of each robot's end effector `seconds` after the first TF in the bag."""
    import rosbag2_py
    import rclpy.time
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    buffer, first = tf2_ros.BufferCore(Duration(seconds=600)), None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            (buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform)(tf, 'bag')
            if first is None and topic == '/tf':
                first = rclpy.time.Time.from_msg(tf.header.stamp)
    out = {}
    for robot in ('blue', 'silver'):
        q = buffer.lookup_transform_core('world', f'oarbot_{robot}/j2n6s300_end_effector',
                                         first+Duration(seconds=seconds)).transform.rotation
        out[robot] = quaternion_matrix(q.x, q.y, q.z, q.w)
    return out


def directions(result):
    import identify_replay as ir
    window = ir.load(Path(result['run']).resolve(), result['window'], 2, 8, 1.)
    g0 = window.grips[0]
    across = .5*(g0[2]+g0[3])-.5*(g0[0]+g0[1]); across[2] = 0.
    inward = -across/np.linalg.norm(across)
    edge = g0[1]-g0[0]; edge[2] = 0.
    return inward, edge/np.linalg.norm(edge)


def world_offsets(values, inward, side):
    out = {}
    for c, name in enumerate(CORNERS):
        toward = inward if c >= 2 else -inward
        out[name] = (-values.get(f'down_{name}', 0.)*np.array([0., 0., 1.]) + values.get(f'in_{name}', 0.)*toward
                     + values.get(f'side_{name}', 0.)*side)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('result', type=Path)
    ap.add_argument('--bag', type=Path, required=True, help='a recording at the same grasp pose, for the tool orientation')
    ap.add_argument('--at-sec', type=float, default=5., help='time into the bag (arms at the grasp pose)')
    ap.add_argument('--cell', type=Path, default=REPO / 'experiments/hardware/cell')
    ap.add_argument('--write', action='store_true')
    a = ap.parse_args()
    result = json.loads(a.result.read_text())
    values = result['calibrated']
    inward, side = directions(result)
    offsets = world_offsets(values, inward, side)
    rotations = tool_rotations(a.bag, a.at_sec)
    # The fitted offsets are corrections to the offsets the recording was made with (its cell/ snapshot; zero for
    # recordings before 2026-11-21): add them, never overwrite (2026-11-21).
    from grip_point_search import recorded_offsets
    recorded = recorded_offsets(Path(result['run']).resolve())
    tool = {r: recorded['tool'][r] + rotations[r].T @ offsets[r] for r in ('blue', 'silver')}
    insets = [round(values.get(f'inset_{c}', 0.), 4) for c in CORNERS]
    hands = recorded['hands'] + np.r_[offsets['left'], offsets['right']]
    for r in ('blue', 'silver'):
        print(f'{r} fabric_grip in the recording: {np.round(recorded["tool"][r], 4).tolist()} m')
    print(f'hand offsets in the recording (left xyz, right xyz): {np.round(recorded["hands"], 4).tolist()} m')
    print(f'directions: in (hands -> robots) {np.round(inward, 3).tolist()}, side (Blue -> Silver) {np.round(side, 3).tolist()}')
    for c in CORNERS:
        print(f'{c:7s} world offset {np.round(offsets[c]*100, 1).tolist()} cm, inset {values.get(f"inset_{c}", 0.)*100:.1f} cm')
    for r in ('blue', 'silver'):
        print(f'{r} fabric_grip translation in j2n6s300_end_effector: {np.round(tool[r], 4).tolist()} m')
    print(f'fusion world_grip_offset_m (left xyz, right xyz): {np.round(hands, 4).tolist()}')
    print(f'twin grip_insets_m (blue, silver, right, left): {insets}')
    if not a.write:
        print('dry run: add --write to change the cell configuration')
        return
    note = f'grip points identified {result.get("run", "")} {result.get("window", "")} (scripts/hw/grip_point_search.py)'
    cal = a.cell/'calibration.yaml'
    text = cal.read_text()
    for r in ('blue', 'silver'):
        pattern = (rf'(- parent: oarbot_{r}/j2n6s300_end_effector\n  child: oarbot_{r}/fabric_grip\n  translation: )\[[^\]]*\]')
        text, n = re.subn(pattern, lambda m: m.group(1)+json.dumps(np.round(tool[r], 4).tolist()), text)
        if n != 1:
            raise SystemExit(f'could not find the {r} fabric_grip transform in {cal}')
    text = text.replace('# PLACEHOLDERS (gate H15): the corner tip held in the fingers, in the tool frame.',
                        f'# IDENTIFIED, not measured (gate H15 still open): the pinch point in the tool frame, {note}.')
    cal.write_text(text)
    fusion = a.cell/'fusion.yaml'
    text, n = re.subn(r'(    world_grip_offset_m: )\[[^\]]*\]', lambda m: m.group(1)+json.dumps(np.round(hands, 4).tolist()),
                      fusion.read_text())
    if n != 1:
        raise SystemExit(f'world_grip_offset_m not found in {fusion}')
    fusion.write_text(text)
    twin = a.cell/'twin.yaml'
    text = twin.read_text()
    if re.search(r'\n  grip_insets_m: ', text):
        text = re.sub(r'(\n  grip_insets_m: )\[[^\]]*\]', lambda m: m.group(1)+json.dumps(insets), text)
    else:
        text = text.replace('\n  substeps:', f'\n  # One held particle per corner at these insets (blue, silver, right, left), {note}.'
                                              f'\n  grip_insets_m: {json.dumps(insets)}\n  substeps:', 1)
    twin.write_text(text)
    print(f'written: {cal}, {fusion}, {twin}')


if __name__ == '__main__':
    main()
