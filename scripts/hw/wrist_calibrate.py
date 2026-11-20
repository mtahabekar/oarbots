#!/usr/bin/env python3
"""Fit a wrist mount from shared RGB-D tag centres and capture-time arm kinematics."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

MIN_FRAMES = 5
MAX_PAIR_SKEW_S = .15
MAX_STILL_SCATTER_M = .015
MAX_VALIDATION_ERROR_M = .020
MAX_FLOOR_TILT_DEG = 2.
MAX_FLOOR_OFFSET_M = .03


def point(transform, position):
    return transform[:3, :3] @ position + transform[:3, 3]


def pose_change(reference, current):
    translation = np.linalg.norm(current[:3, 3] - reference[:3, 3])
    rotation = Rotation.from_matrix(reference[:3, :3].T @ current[:3, :3]).magnitude()
    return float(translation), float(np.degrees(rotation))


def distinct_wrist_pose(reference, current):
    distance, angle = pose_change(reference, current)
    return distance >= .04 or angle >= 5.


def camera_calibration(path):
    cell = yaml.safe_load(path.read_text())
    world = cell['world_frame']
    calibration = {}
    for row in cell['transforms']:
        if row['parent'] == world:
            frame = np.eye(4)
            frame[:3, :3] = Rotation.from_quat(row['rotation_xyzw']).as_matrix()
            frame[:3, 3] = row['translation']
            calibration[row['child']] = frame
    return calibration


def read_capture(pose, camera):
    path = pose / f'{camera}.jsonl'
    if not path.exists():
        return []
    samples = [json.loads(line) for line in path.read_text().splitlines() if line]
    if any(sample['camera'] != camera for sample in samples):
        raise ValueError(f'{path}: unexpected camera name')
    return sorted(samples, key=lambda sample: sample['stamp'])


def read_station(pose, robot, surveyed):
    wrist_camera = f'{robot}_wrist'
    ns = f'oarbot_{robot}'
    overhead = read_capture(pose, 'overhead')
    wrist = read_capture(pose, wrist_camera)
    if not overhead or not wrist:
        return [], 'missing overhead or wrist detections'
    expected_frames = {'nuc/rgb_camera_link', f'{ns}/rgb_camera_link'}
    if {sample['frame'] for sample in overhead + wrist} != expected_frames:
        raise ValueError(f'{pose}: unexpected RGB optical frames')
    sizes = {tuple(sorted(sample.get('paper_widths_m', {'glove': sample['tag_size_m']}).items()))
             for sample in overhead + wrist}
    if len(sizes) != 1:
        raise ValueError(f'{pose}: target mode or measured tag widths differ across cameras')
    target = sizes.pop()
    transforms = {key: np.asarray(matrix) for key, matrix in wrist[0]['transforms'].items()}
    for sample in wrist:
        distance, angle = pose_change(transforms['root_from_tool'], np.asarray(sample['transforms']['root_from_tool']))
        if distance > .003 or angle > .3:
            return [], 'arm moved during capture'
    # Pair only overlapping frames. Using each overhead stamp once avoids counting a
    # dropped or slower overhead stream as repeated evidence for the same wrist view.
    overhead_stamps = np.array([sample['stamp'] for sample in overhead])
    by_tag = {}
    used = set()
    for sample in wrist:
        nearest = int(np.argmin(abs(overhead_stamps - sample['stamp'])))
        skew = abs(overhead_stamps[nearest] - sample['stamp'])
        if nearest in used or skew > MAX_PAIR_SKEW_S:
            continue
        reference = overhead[nearest]
        shared = sample['positions_m'].keys() & reference['positions_m'].keys()
        if not shared:
            continue
        used.add(nearest)
        world_from_rgb = surveyed['nuc/camera_base'] @ np.asarray(reference['transforms']['base_from_rgb'])
        world_from_tool = surveyed[f'{ns}/base_link'] @ np.asarray(sample['transforms']['root_from_tool'])
        tool_from_world = np.linalg.inv(world_from_tool)
        for tag in shared:
            world_position = point(world_from_rgb, reference['positions_m'][tag])
            points = (np.asarray(sample['positions_m'][tag]), point(tool_from_world, world_position), world_position)
            by_tag.setdefault(tag, []).append((sample['stamp'], points))
    observations = []
    for tag, pairs in sorted(by_tag.items()):
        if len(pairs) < MIN_FRAMES:
            continue
        stamps, points = zip(*pairs)
        camera_points, tool_points, world_points = np.asarray(points).transpose(1, 0, 2)
        camera_error = np.linalg.norm(camera_points - np.median(camera_points, axis=0), axis=1)
        world_error = np.linalg.norm(world_points - np.median(world_points, axis=0), axis=1)
        stable = np.maximum(camera_error, world_error) <= MAX_STILL_SCATTER_M
        clean = int(stable.sum())
        if clean < max(MIN_FRAMES, np.ceil(2 * len(pairs) / 3)) or np.ptp(np.asarray(stamps)[stable]) < 1.:
            continue
        observations.append(dict(pose=pose.name, tag=int(tag), pairs=clean, available_pairs=len(pairs),
                                 camera_m=np.median(camera_points[stable], axis=0),
                                 tool_m=np.median(tool_points[stable], axis=0),
                                 world_m=np.median(world_points[stable], axis=0), target=target, **transforms))
    return observations, ('usable' if observations else 'keep one shared tag steady with depth in both views')


def register(camera, tool):
    """Least-squares proper rigid transform mapping wrist RGB coordinates into the tool frame."""
    a, b = camera.mean(axis=0), tool.mean(axis=0)
    span = np.linalg.svd(camera - a, compute_uv=False) / np.sqrt(len(camera))
    if np.any(span[:2] < [.07, .025]) or span[2] < .01:
        raise ValueError('Spread the glove placements in width, depth and height for a well-conditioned fit')
    u, _, vt = np.linalg.svd((camera - a).T @ (tool - b))
    proper = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
    transform = np.eye(4)
    transform[:3, :3] = vt.T @ proper @ u.T
    transform[:3, 3] = b - transform[:3, :3] @ a
    return transform


def inspect_station(pose, robot, surveyed):
    observations, status = read_station(pose, robot, surveyed)
    tags = ', '.join(f'tag {row["tag"]} ({row["pairs"]}/{row["available_pairs"]} pairs)'
                     for row in observations)
    print(f'{robot} {pose.name}: {status}' + (f', {tags}' if tags else ''))
    return observations


def floor_capture(robot, background, surveyed):
    if __package__:
        from . import obstacle_detect as detector
    else:
        import obstacle_detect as detector

    old_calibration = background / 'calibration.yaml'
    if old_calibration.is_file():
        original_world = camera_calibration(old_calibration)
        for root in ('nuc/camera_base', f'oarbot_{robot}/base_link'):
            if not np.array_equal(surveyed[root], original_world[root]):
                raise ValueError('The floor reference used a different camera or robot-base survey')
    lo, hi = np.array([-.7, -.75, .02]), np.array([.7, .7, 1.3])
    centre = ((lo + hi) / 2)[:2]
    with np.load(background / 'overhead.npz') as overhead, np.load(background / f'{robot}_wrist.npz') as capture:
        reference, _ = detector.fit_floor(overhead, lo, hi, surveyed)
        snap = dict(capture)
    if str(snap['frame_id']) != f'oarbot_{robot}/depth_camera_link':
        raise ValueError('The floor capture uses the wrong wrist depth frame')
    old, _ = detector.fit_floor(snap, lo, hi, surveyed)
    if reference is None or old is None:
        raise ValueError('Could not establish the floor in the saved capture')
    return detector, snap, lo, hi, centre, reference, old


def fit_floor_alignment(robot, background, surveyed, anchor, transform, camera, tool):
    """Fit tag centres and a measured floor plane together, using one saved arm pose."""
    from scipy.optimize import least_squares

    detector, snap, _, _, centre, reference, old = floor_capture(robot, background, surveyed)
    tool_from_base = anchor['tool_from_mount'] @ anchor['mount_from_base']
    tool_from_depth = tool_from_base @ anchor['base_from_depth']
    tool_from_rgb = tool_from_base @ anchor['base_from_rgb']
    root = surveyed[f'oarbot_{robot}/base_link']
    world_from_tool = root @ snap['T_root_depth'] @ np.linalg.inv(tool_from_depth)
    rgb_from_world = np.linalg.inv(world_from_tool @ tool_from_rgb)

    def normal(plane):
        vector = np.r_[-plane[:2], 1.]
        return vector / np.linalg.norm(vector)

    source_normal, reference_normal = normal(old), normal(reference)
    source_point = np.r_[centre, detector.plane_height(old, centre)]
    reference_offset = reference_normal @ np.r_[centre, detector.plane_height(reference, centre)]

    def residual(params):
        candidate = np.eye(4)
        candidate[:3, :3] = Rotation.from_rotvec(params[:3]).as_matrix()
        candidate[:3, 3] = params[3:]
        world_correction = world_from_tool @ candidate @ rgb_from_world
        centre_error = camera @ candidate[:3, :3].T + params[3:] - tool
        tilt_error = world_correction[:3, :3] @ source_normal - reference_normal
        height_error = reference_normal @ point(world_correction, source_point) - reference_offset
        return np.r_[centre_error.ravel() / .01, tilt_error / np.radians(.5), height_error / .01]

    initial = np.r_[Rotation.from_matrix(transform[:3, :3]).as_rotvec(), transform[:3, 3]]
    solved = least_squares(residual, initial)
    if not solved.success:
        raise ValueError('Could not solve the joint tag and floor fit')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(solved.x[:3]).as_matrix()
    result[:3, 3] = solved.x[3:]
    return result


def replay_floor(robot, background, surveyed, anchor, mount_from_base):
    """Apply only the candidate mount correction to the original raw depth recording."""
    detector, snap, lo, hi, centre, reference, old = floor_capture(robot, background, surveyed)
    nominal = anchor['tool_from_mount'] @ anchor['mount_from_base'] @ anchor['base_from_depth']
    candidate = anchor['tool_from_mount'] @ mount_from_base @ anchor['base_from_depth']
    snap['T_root_depth'] = snap['T_root_depth'] @ np.linalg.solve(nominal, candidate)
    new, _ = detector.fit_floor(snap, lo, hi, surveyed)
    if new is None:
        raise ValueError('Could not establish the floor in the candidate replay')
    old_tilt, old_height = detector.floor_difference(old, reference, centre)
    tilt, height = detector.floor_difference(new, reference, centre)
    accepted = tilt <= MAX_FLOOR_TILT_DEG and height <= MAX_FLOOR_OFFSET_M
    print(f'{robot} floor tilt {old_tilt:.2f} → {tilt:.2f} deg, '
          f'height difference {100*old_height:.2f} → {100*height:.2f} cm. Passed: {accepted}')
    return dict(background=str(background), nominal_tilt_deg=old_tilt, nominal_height_m=old_height,
                candidate_tilt_deg=tilt, candidate_height_m=height, passed=accepted)


def saved_depth_correction(session, robot):
    """Map a raw nominal wrist depth pose to the qualified camera-mount candidate."""
    report = json.loads((session / f'{robot}_result.json').read_text())
    params = report['candidate']
    prefix = f'oarbot_{robot}/'
    if (report['robot'] != robot or report['qualified'] is not True
            or params['parent'] != prefix + 'right_mount' or params['child'] != prefix + 'camera_base'):
        raise ValueError(f'{robot}: expected a qualified wrist-mount calibration')
    poses = dict.fromkeys(row['pose'] for row in report['fit'] + report['held_out'])
    for pose in poses:
        records = read_capture(session / pose, f'{robot}_wrist')
        anchor = next((row['transforms'] for row in records if 'base_from_depth' in row['transforms']), None)
        if anchor is not None:
            break
    else:
        raise ValueError(f'{robot}: calibration session lacks a saved depth optical transform')
    fitted = np.eye(4)
    fitted[:3, :3] = Rotation.from_quat(params['rotation_xyzw']).as_matrix()
    fitted[:3, 3] = params['translation']
    arm = np.asarray(anchor['tool_from_mount'])
    depth = np.asarray(anchor['base_from_depth'])
    nominal = arm @ np.asarray(anchor['mount_from_base']) @ depth
    return np.linalg.solve(nominal, arm @ fitted @ depth)


def fit(robot, session, floor_reference=None, fit_floor=None):
    surveyed = camera_calibration(session / 'calibration.yaml')
    subsets = {}
    for prefix in ('fit', 'check'):
        stations = []
        for pose in sorted(session.glob(f'{prefix}_*')):
            if pose.is_dir():
                observations = inspect_station(pose, robot, surveyed)
                if observations:
                    stations.append(observations)
        subsets[prefix] = stations
    if len(subsets['fit']) < 5 or len(subsets['check']) < 3:
        raise ValueError('Collect at least five usable fit_* holds and three usable check_* holds')
    training = [row for station in subsets['fit'] for row in station]
    validation = [row for station in subsets['check'] for row in station]
    anchor = training[0]
    if len({row['target'] for row in training + validation}) != 1:
        raise ValueError('Use one target mode and measured tag widths per session')
    paper_target = anchor['target'][0][0] != 'glove'
    if paper_target and not any(
            distinct_wrist_pose(anchor['root_from_tool'], row['root_from_tool']) for row in training):
        raise ValueError('Collect paper fitting holds at more than one arm pose before checking a third pose')
    for name in ('tool_from_mount', 'mount_from_base', 'base_from_rgb'):
        for row in training + validation:
            distance, angle = pose_change(anchor[name], row[name])
            if distance > .001 or angle > .1:
                raise ValueError(f'TF mount chain changed during this session ({name})')
    camera = np.array([row['camera_m'] for row in training])
    tool = np.array([row['tool_m'] for row in training])
    tool_from_rgb = register(camera, tool)
    if floor_reference is not None or fit_floor is not None:
        factory = next((row for row in training + validation if 'base_from_depth' in row), None)
        if factory is None:
            raise ValueError('Floor replay requires a capture with the depth optical transform')
    if fit_floor is not None:
        if floor_reference is None or floor_reference.resolve() == fit_floor.resolve():
            raise ValueError('The floor fit needs a separate empty depth capture for validation')
        tool_from_rgb = fit_floor_alignment(robot, fit_floor, surveyed, factory, tool_from_rgb, camera, tool)
    nominal = anchor['tool_from_mount'] @ anchor['mount_from_base'] @ anchor['base_from_rgb']
    mount_from_base = np.linalg.solve(anchor['tool_from_mount'], tool_from_rgb) @ np.linalg.inv(anchor['base_from_rgb'])

    def errors(rows):
        result = []
        for row in rows:
            expected = row['tool_m']
            distance, angle = pose_change(anchor['root_from_tool'], row['root_from_tool'])
            result.append(dict(pose=row['pose'], tag=row['tag'], pairs=row['pairs'],
                               available_pairs=row['available_pairs'],
                               wrist_change_from_first_fit_m=distance,
                               wrist_change_from_first_fit_deg=angle,
                               error_m=float(np.linalg.norm(point(tool_from_rgb, row['camera_m']) - expected)),
                               nominal_error_m=float(np.linalg.norm(point(nominal, row['camera_m']) - expected))))
        return result

    held_out = errors(validation)
    p95 = float(np.percentile([row['error_m'] for row in held_out], 95))
    different_arm_pose = any(
        all(distinct_wrist_pose(train['root_from_tool'], row['root_from_tool']) for train in training)
        for row in validation
    )
    floor = None
    if floor_reference is not None:
        floor = replay_floor(robot, floor_reference, surveyed, factory, mount_from_base)
    floor_passed = floor['passed'] if floor is not None else not paper_target
    qualified = p95 <= MAX_VALIDATION_ERROR_M and different_arm_pose and floor_passed
    candidate = dict(parent=f'oarbot_{robot}/right_mount', child=f'oarbot_{robot}/camera_base',
                     translation=mount_from_base[:3, 3].tolist(),
                     rotation_xyzw=Rotation.from_matrix(mount_from_base[:3, :3]).as_quat().tolist())
    report = dict(robot=robot, qualified=qualified, target_widths_m=dict(anchor['target']),
                  candidate=candidate, fit=errors(training),
                  held_out=held_out, held_out_p95_m=p95, held_out_at_different_arm_pose=different_arm_pose,
                  floor_fit_background=str(fit_floor) if fit_floor else None, floor_replay=floor)
    path = session / f'{robot}_result.json'
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{robot} held-out p95: {100*p95:.1f} cm. Qualified: {qualified}. Report: {path}')
    if not different_arm_pose:
        print('Repeat withheld measurements at a different stationary wrist pose for qualification.')
    if paper_target and floor is None:
        print('Replay the matching empty depth capture with --floor-reference for qualification.')
    return qualified


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot', choices=('silver', 'blue'))
    parser.add_argument('session', type=Path)
    parser.add_argument('--inspect', metavar='HOLD', help='check one capture without fitting')
    parser.add_argument('--floor-reference', type=Path, metavar='BACKGROUND',
                        help='verify floor alignment using matching empty depth captures')
    parser.add_argument('--fit-floor', type=Path, metavar='BACKGROUND',
                        help='fit an empty floor alongside the tags, with a separate --floor-reference')
    args = parser.parse_args()
    try:
        if args.inspect:
            survey = camera_calibration(args.session / 'calibration.yaml')
            return 0 if inspect_station(args.session / args.inspect, args.robot, survey) else 1
        qualified = fit(args.robot, args.session, args.floor_reference, args.fit_floor)
    except (OSError, KeyError, ValueError) as exc:
        parser.exit(2, f'{exc}\n')
    return 0 if qualified else 1


if __name__ == '__main__':
    raise SystemExit(main())
