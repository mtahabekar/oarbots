#!/usr/bin/env python3
"""Reproduce the grasp-pose capture and motion audit with the deployed perception code."""

import argparse
import csv
import gzip
import json
from collections import Counter
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import yaml

from fabric_perception.core import Camera, DepthProcessor, DepthWorkspace, Segmenter, boundaries, depth_metres
from glove_check import NAMESPACES, detector_from, load_params
from human_hand_tracking.fusion import rotation_matrix


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
OVERHEAD_ROI = [.48, .25, .80, .65]


def read_camera(run):
    info = json.loads((run / 'camera_info.json').read_text())
    return Camera(info['width'], info['height'], info['k'], info['d'])


def read_frame(run, index, camera, detector, params):
    stem = run / f'frame_{index:03d}'
    bgr = cv2.imread(f'{stem}_color.png')
    depth = depth_metres(cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED), '16UC1')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    gloves, _ = detector.detect(
        bgr, camera.k, camera.distortion, depth=depth if params['tag_use_depth'] else None, hsv=hsv,
    )
    return bgr, depth, hsv, gloves


def processors(params):
    segmenter = Segmenter(
        cloth_max_value=params['cloth_max_value'], cloth_max_saturation=params['cloth_max_saturation'],
        min_component_area=params['min_component_area'], roi=params['workspace_roi'],
        cloth_dark_value=params.get('cloth_dark_value', -1),
    )
    depth_processor = DepthProcessor(
        params['min_depth_m'], params['max_depth_m'], params['depth_edge_threshold_m'],
        params['depth_noise_floor_m'], params['depth_noise_scale'], params['sample_stride'],
        params['max_depth_samples'],
    )
    return segmenter, depth_processor


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

    camera = read_camera(run)
    params = load_params(CELL / f'perception_{camera_id}.yaml')
    detector = detector_from(params)
    segmenter, depth_processor = processors(params)
    candidates = {}
    if camera_id == 'overhead':
        for name, saturation, value in (
            ('roi_only', params['cloth_max_saturation'], params['cloth_max_value']),
            ('roi_and_hsv', 60, 90),
        ):
            candidates[name] = Segmenter(
                cloth_max_value=value, cloth_max_saturation=saturation,
                min_component_area=params['min_component_area'], roi=OVERHEAD_ROI,
            )
        summary['overhead_candidate_settings'] = {
            name: {'roi': model.roi, 'max_saturation': model.max_s, 'max_value': model.max_v}
            for name, model in candidates.items()
        }
    frames, tiles = {}, []
    for index in (5, 22, 38):
        stem = run / f'frame_{index:03d}'
        bgr, depth, hsv, glove_mask = read_frame(run, index, camera, detector, params)
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


def workspace_summary(capture, label_path=None):
    world = json.loads((SESSION.parent / '2026-11-17_world' / 'overhead_extrinsics.json').read_text())
    world = world['world_to_nuc_rgb_camera_link']
    transform = rotation_matrix(world['rotation_xyzw']), world['translation']
    annotations = json.loads((label_path or SESSION / 'workspace_labels.json').read_text())
    profiles = yaml.safe_load((CELL / 'perception_workspace.yaml').read_text())
    summary = {'annotation_margin_px': 12, 'cameras': {}}
    tiles = []
    # Exclude a tolerance band around the hand-drawn outline from pixel scores.
    margin = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    for name in RUNS:
        if label_path is None:
            run = capture / RUNS[name]
        else:
            runs = sorted(path for path in capture.rglob(f'{name}_*') if (path / 'camera_info.json').is_file())
            if len(runs) != 1:
                raise ValueError(f'Expected one {name} recording in {capture}, found {len(runs)}')
            run = runs[0]
        camera = read_camera(run)
        params = load_params(CELL / f'perception_{name}.yaml')
        detector = detector_from(params)
        overhead = name == 'overhead'
        overrides = profiles[f'{NAMESPACES[name]}/fabric_perception']['ros__parameters']
        colour = {key: overrides[key] for key in ('workspace_roi', 'cloth_max_saturation', 'cloth_max_value')
                  if key in overrides}
        geometry = {key: value for key, value in overrides.items() if key not in colour and key != 'cloth_dark_value'}
        dark_value = overrides.get('cloth_dark_value', 60)
        workspace = DepthWorkspace(geometry.get('workspace_depth_range_m'), geometry.get('workspace_box_m'),
                                   geometry['workspace_depth_fill_px'])
        variants = {
            'baseline': (params, False),
            'colour_roi': ({**params, **colour}, False),
            'geometry': ({**params, **colour}, True),
            'dark_geometry': ({**params, **colour, 'cloth_dark_value': dark_value}, True),
        }
        candidates = {label: (*processors(settings), use_gate) for label, (settings, use_gate) in variants.items()}
        frames = {}
        for index, polygons in annotations[name].items():
            bgr, depth, hsv, gloves = read_frame(run, int(index), camera, detector, params)
            truth = np.zeros(depth.shape, np.uint8)
            if polygons:
                cv2.fillPoly(truth, [np.asarray(polygon, np.int32) * 4 for polygon in polygons], 255)
            inside = cv2.erode(truth, margin) > 0
            outside = cv2.dilate(truth, margin) == 0
            # Warm the ray cache outside the timed measurements.
            workspace.mask(depth, camera, transform if overhead else None)
            metrics = {}
            row = [cv2.resize(bgr, (480, 270))]
            cv2.polylines(row[0], [np.asarray(polygon, np.int32) for polygon in polygons], True, (0, 255, 255), 1)
            for label, (segmenter, processor, use_gate) in candidates.items():
                times = []
                for _ in range(3):
                    start = perf_counter()
                    gate = workspace.mask(depth, camera, transform if overhead else None) if use_gate else None
                    sheet, _, confidence, occlusion, roi = segmenter.infer(bgr, gloves, hsv=hsv, depth_roi=gate)
                    valid, _, pixels, _, _ = processor.process(depth, sheet, camera)
                    contours = boundaries(sheet, confidence, occlusion, roi, params['max_boundary_points'])
                    times.append((perf_counter() - start) * 1000)
                predicted = sheet > 0
                tp, fp = int((predicted & inside).sum()), int((predicted & outside).sum())
                metrics[label] = {
                    'precision': round(tp / (tp + fp), 4) if inside.any() and tp + fp else None,
                    'recall': round(tp / int(inside.sum()), 4) if inside.any() else None,
                    'false_positive_pixels': fp,
                    'accepted_depth_on_cloth': round(float(valid[inside].mean()), 4) if inside.any() else None,
                    'sampled_depth_points': len(pixels),
                    'supported_boundary_points': sum(int(np.count_nonzero(weights)) for _, weights, _ in contours),
                    'total_boundary_points': sum(len(weights) for _, weights, _ in contours),
                    'processing_ms_median_of_3': round(float(np.median(times)), 2),
                }
                view = bgr.copy()
                view[predicted] = (0.4 * view[predicted] + [153, 0, 153]).astype(np.uint8)
                row.append(cv2.resize(view, (480, 270)))
            reference = 'visible cloth outline' if polygons else 'no cloth (reference)'
            for image, label in zip(row, (reference, *candidates)):
                cv2.putText(image, f'{name} {index}  {label}', (7, 260), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (255, 255, 255), 1)
            tiles.append(np.hstack(row))
            frames[index] = {
                'label_interior_pixels': int(inside.sum()),
                'raw_depth_on_cloth': round(float((depth[inside] > 0).mean()), 4) if inside.any() else None,
                'variants': metrics,
            }
        summary['cameras'][name] = {'colour_settings': colour, 'geometry_settings': geometry,
                                   'dark_value': dark_value, 'frames': frames}
    return summary, np.vstack(tiles)


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
    parser.add_argument('--workspace', action='store_true', help='Score the labelled workspace comparison')
    parser.add_argument('--workspace-labels', type=Path, help='Use new annotations and camera runs for the comparison')
    parser.add_argument('--image', type=Path, help='Optional contact sheet to write')
    args = parser.parse_args()
    if args.workspace or args.workspace_labels:
        result, image = workspace_summary(args.capture_dir, args.workspace_labels)
    else:
        cameras, tiles = {}, []
        for camera_id in RUNS:
            cameras[camera_id], tile = capture_summary(args.capture_dir, camera_id)
            tiles.append(tile)
        result = {'cameras': cameras, 'cartesian_attempts': cartesian_summary()}
        image = np.vstack(tiles)
    if args.image and not cv2.imwrite(str(args.image), image):
        raise OSError(f'Could not write {args.image}')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
