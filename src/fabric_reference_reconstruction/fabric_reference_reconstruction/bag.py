"""Extract capture-time factors from MCAP/sqlite bags; never read estimated_state."""
import argparse
from dataclasses import replace
from pathlib import Path
import json
import numpy as np
from fabric_state_estimator.config import load_config
from fabric_state_estimator.data import Frame, Grip, Force, save_sequence
from fabric_state_estimator.ros_adapters import nanoseconds, view_from_message
from fabric_state_estimator.transforms import TransformHistory


def read_bag(path, storage_id):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id), rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    allowed = {'tf2_msgs/msg/TFMessage', 'fabric_state_msgs/msg/FabricObservations',
               'fabric_state_msgs/msg/GloveObservations', 'geometry_msgs/msg/WrenchStamped'}
    classes = {topic: get_message(kind) for topic, kind in types.items() if kind in allowed}
    while reader.has_next():
        topic, payload, recorded_ns = reader.read_next()
        if topic in classes:
            yield topic, deserialize_message(payload, classes[topic])


def extract(messages, mesh, runtime, max_frames=120, stride=1):
    """Two passes in memory for bounded calibration sequences; TF may arrive late."""
    if max_frames < 2 or stride < 1:
        raise ValueError('Need max_frames>=2 and stride>=1')
    history = TransformHistory()
    observations, gloves, forces = {}, {}, [[], []]
    for topic, message in messages:
        if topic in ('/tf', '/tf_static'):
            for transform in message.transforms:
                t, q = transform.transform.translation, transform.transform.rotation
                history.add(transform.header.frame_id, transform.child_frame_id, nanoseconds(transform.header.stamp),
                            [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], topic == '/tf_static')
        elif topic in runtime.observation_topics:
            observations[(message.camera_id, nanoseconds(message.header.stamp))] = message
        elif hasattr(message, 'hands') and hasattr(message, 'camera_id'):
            gloves[(message.camera_id, nanoseconds(message.header.stamp))] = message
        elif runtime.use_forces and topic in runtime.force_topics:
            forces[runtime.force_topics.index(topic)].append(message)
    groups = []
    for message in sorted(observations.values(), key=lambda m: nanoseconds(m.header.stamp)):
        capture = nanoseconds(message.header.stamp)
        if not groups or capture-nanoseconds(groups[-1][0].header.stamp) > runtime.camera_sync_tolerance_sec*1e9:
            groups.append([])
        groups[-1].append(message)
    frames, dropped = [], []
    for group in groups[::stride]:
        capture = max(nanoseconds(m.header.stamp) for m in group)
        if len(frames) >= max_frames:
            break
        try:
            views, grips, reactions = [], [], []
            for corner, robot_frame in enumerate(runtime.robot_grip_frames):
                transform = history.lookup(runtime.world_frame, robot_frame, capture)
                grips.append(Grip(corner, transform[:3, 3], np.eye(3)*runtime.robot_position_std_m**2))
            for message in group:
                source_capture = nanoseconds(message.header.stamp)
                transform = history.lookup(runtime.world_frame, message.header.frame_id, source_capture)
                views.append(view_from_message(message, transform))
                glove = gloves.get((message.camera_id, source_capture))
                if glove is not None:
                    glove_transform = history.lookup(runtime.world_frame, glove.header.frame_id, source_capture)
                    for hand in glove.hands:
                        if hand.hand_id not in ('human_left', 'human_right'):
                            continue
                        corner = 2 if hand.hand_id == 'human_left' else 3
                        p = hand.position; rotation = glove_transform[:3, :3]
                        # Per-view tag covariance excludes camera extrinsic error
                        # (online fusion adds it); add it here too.
                        grips.append(Grip(corner, rotation @ [p.x, p.y, p.z]+glove_transform[:3, 3],
                            rotation @ np.asarray(hand.covariance).reshape(3, 3) @ rotation.T
                            + np.eye(3)*(((capture-source_capture)/1e9)**2+runtime.glove_extrinsic_std_m**2)))
            # Missing hands are allowed after initialization; their states remain
            # inferred under soft dynamics. No filtered hand or fabric trajectory is fitted.
            for corner, samples in enumerate(forces):
                eligible = [sample for sample in samples if 0 <= (capture-nanoseconds(sample.header.stamp))/1e9 <= runtime.force_max_age_sec]
                if eligible:
                    sample = max(eligible, key=lambda m: nanoseconds(m.header.stamp))
                    transform = history.lookup(runtime.world_frame, sample.header.frame_id, nanoseconds(sample.header.stamp))
                    f = sample.wrench.force
                    reactions.append(Force(corner, runtime.force_on_sheet_sign*(transform[:3, :3] @ [f.x, f.y, f.z]), runtime.force_stddev_n))
            frame = Frame(capture, views, grips, reactions)
            if not frames:
                frame.grip_targets()  # first frame must identify all four corners
            frames.append(frame)
        except ValueError as error:
            dropped.append({'time_ns': capture, 'reason': str(error)})
    if len(frames) < 2:
        raise ValueError('Not enough usable frames; check raw glove topics, grip frames and capture-time TF')
    return frames, dropped


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('config'); parser.add_argument('output')
    parser.add_argument('--storage-id', default='mcap', choices=['mcap', 'sqlite3'])
    parser.add_argument('--max-frames', type=int, default=120); parser.add_argument('--stride', type=int, default=1)
    options = parser.parse_args(args)
    mesh, physics, _, runtime, _, _ = load_config(options.config)
    frames, dropped = extract(read_bag(options.bag, options.storage_id), mesh, runtime, options.max_frames, options.stride)
    provenance = {'bag': str(Path(options.bag).resolve()), 'config_text': Path(options.config).read_text(),
                  'dropped': dropped, 'human_grips': 'raw per-view glove observations; holding assumed',
                  'input': 'perception factors; no online fabric state'}
    save_sequence(options.output, mesh, physics, frames, provenance)
    print(json.dumps({'frames': len(frames), 'dropped': len(dropped)}))


if __name__ == '__main__':
    main()
