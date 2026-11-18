#!/usr/bin/env python3
"""Reproduce the grasp-pose capture and motion audit with the deployed perception code."""

import argparse
import csv
import gzip
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from fabric_perception.core import Camera, DepthProcessor, Segmenter, boundaries, depth_metres
from glove_check import detector_from, load_params


SESSION = Path(__file__).resolve().parent
CELL = SESSION.parents[1] / 'cell'
RUNS = {
    'overhead': 'overhead_20261118_172144',
    'blue_wrist': 'blue_wrist_20261118_172143',
    'silver_wrist': 'silver_wrist_20261118_172143',
}
# Probe rectangles from the original diagnostics, in raw RGB coordinates.
PROBES = {
    'overhead': {5: (1100, 360, 1340, 500), 22: (1100, 440, 1230, 580)},
    'blue_wrist': {5: (300, 480, 1600, 1020), 22: (450, 700, 1350, 1020)},
    'silver_wrist': {5: (150, 480, 1500, 1020), 22: (200, 760, 1100, 1060)},
}


def capture_summary(capture, camera_id):
    run = capture / RUNS[camera_id]
    rows = [json.loads(line) for line in (run / 'frames.jsonl').read_text().splitlines()]
    pairs = [row['saved'] for row in rows if 'saved' in row]
    for name in pairs:
        for suffix in ('color.png', 'depth.png', 'annotated.jpg'):
            if not (run / f'{name}_{suffix}').is_file():
                raise FileNotFoundError(run / f'{name}_{suffix}')
    detections = Counter(tag for row in rows for tag in row['detected'])
    misses = Counter(
        (tag['id'], tag['why']) for row in rows for tag in row['tags']
        if tag['id'] in (20, 21) and not tag['detected']
    )
    summary = {
        'diagnostic_frames': len(rows),
        'saved_rgbd_pairs': len(pairs),
        'tag_detections': {str(tag): detections[tag] for tag in (20, 21)},
        'both_detected_frames': sum(len(row['detected']) == 2 for row in rows),
        'tag_misses_in_full_frame_search': {
            str(tag): {reason: count for (key, reason), count in sorted(misses.items()) if key == tag}
            for tag in (20, 21)
        },
    }

    info = json.loads((run / 'camera_info.json').read_text())
    camera = Camera(info['width'], info['height'], info['k'], info['d'])
    params = load_params(CELL / f'perception_{camera_id}.yaml')
    detector = detector_from(params)
    segmenter = Segmenter(
        cloth_max_value=params['cloth_max_value'], cloth_max_saturation=params['cloth_max_saturation'],
        min_component_area=params['min_component_area'], roi=params['workspace_roi'],
    )
    candidates = {}
    if camera_id == 'overhead':
        roi = [.48, .25, .80, .65]  # contains the observed overhead cloth region
        for name, saturation, value in (
            ('roi_only', params['cloth_max_saturation'], params['cloth_max_value']),
            ('roi_and_hsv', 60, 90),
        ):
            candidates[name] = Segmenter(
                cloth_max_value=value, cloth_max_saturation=saturation,
                min_component_area=params['min_component_area'], roi=roi,
            )
        summary['overhead_candidate_settings'] = {
            name: {'roi': model.roi, 'max_saturation': model.max_s, 'max_value': model.max_v}
            for name, model in candidates.items()
        }
    depth_processor = DepthProcessor(
        params['min_depth_m'], params['max_depth_m'], params['depth_edge_threshold_m'],
        params['depth_noise_floor_m'], params['depth_noise_scale'], params['sample_stride'],
        params['max_depth_samples'],
    )
    frames, tiles = {}, []
    for index in (5, 22, 38):
        stem = run / f'frame_{index:03d}'
        bgr = cv2.imread(f'{stem}_color.png')
        depth = depth_metres(cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED), '16UC1')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        glove_mask, _ = detector.detect(
            bgr, camera.k, camera.distortion, depth=depth if params['tag_use_depth'] else None, hsv=hsv,
        )
        sheet, _, confidence, occlusion, roi = segmenter.infer(bgr, glove_mask, hsv=hsv)
        _, _, _, points, depth_fraction = depth_processor.process(depth, sheet, camera)
        contours = boundaries(sheet, confidence, occlusion, roi, params['max_boundary_points'])
        frame = {
            'elapsed_s': next(row['t'] for row in rows if row.get('saved') == stem.name),
            'baseline_sheet_pixels': int(np.count_nonzero(sheet)),
            'baseline_depth_fraction': round(depth_fraction, 4),
            'baseline_depth_samples': len(points),
            'baseline_weighted_boundary_samples': sum(int(np.count_nonzero(w)) for _, w, _ in contours),
        }
        if index in PROBES[camera_id]:
            x0, y0, x1, y1 = PROBES[camera_id][index]
            probe = np.s_[y0:y1, x0:x1]
            probe_depth = depth[probe]
            frame['probe_xyxy_px'] = [x0, y0, x1, y1]
            frame['probe_nonzero_depth_fraction'] = round(float((probe_depth > 0).mean()), 4)
            frame['probe_range_m_p01_p50_p99'] = np.percentile(
                probe_depth[probe_depth > 0], [1, 50, 99],
            ).round(3).tolist()
            frame['probe_baseline_mask_fraction'] = round(float((sheet[probe] > 0).mean()), 4)
        for name, model in candidates.items():
            candidate, *_ = model.infer(bgr, glove_mask, hsv=hsv)
            frame[f'{name}_mask_pixels'] = int(np.count_nonzero(candidate))
            if index in PROBES[camera_id]:
                frame[f'{name}_probe_mask_fraction'] = round(float((candidate[probe] > 0).mean()), 4)
        frames[str(index)] = frame
        bgr[sheet > 0] = (0.4 * bgr[sheet > 0] + 0.6 * np.array([255, 0, 255])).astype(np.uint8)
        tile = cv2.resize(bgr, (640, 360))
        label = f'{camera_id} frame {index}: {frame["baseline_sheet_pixels"]:,} mask pixels'
        cv2.putText(tile, label, (8, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        tiles.append(tile)
    summary['baseline_frames'] = frames
    return summary, np.hstack(tiles)


def cartesian_summary():
    summary = {}
    for path in sorted(SESSION.glob('motion/*/grasp_drive/*_samples.csv.gz')):
        with gzip.open(path, 'rt', newline='') as file:
            reader = csv.reader(file)
            next(reader)
            data = np.asarray(list(reader), dtype=float)
        speed = np.linalg.norm(data[:, 1:4], axis=1)
        xyz = data[:, 4:7]
        joints = data[:, 8:14]
        summary[path.name.removesuffix('_samples.csv.gz')] = {
            'samples': len(data),
            'elapsed_s': round(float(data[-1, 0] - data[0, 0]), 3),
            'nonzero_guard_samples': int((speed > 0).sum()),
            'maximum_guard_speed_mps': round(float(speed.max()), 5),
            'maximum_tool_displacement_mm': round(float(np.linalg.norm(xyz - xyz[0], axis=1).max() * 1000), 3),
            'maximum_joint_delta_rad': round(float(np.abs(joints - joints[0]).max()), 6),
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--image', type=Path, help='Optional baseline contact sheet to write')
    args = parser.parse_args()
    cameras, tiles = {}, []
    for camera_id in RUNS:
        cameras[camera_id], tile = capture_summary(args.capture_dir, camera_id)
        tiles.append(tile)
    if args.image and not cv2.imwrite(str(args.image), np.vstack(tiles)):
        raise OSError(f'Could not write {args.image}')
    print(json.dumps({'cameras': cameras, 'cartesian_attempts': cartesian_summary()}, indent=2))


if __name__ == '__main__':
    main()
