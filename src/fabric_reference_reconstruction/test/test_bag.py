import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Point, Point32
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from fabric_state_msgs.msg import FabricObservations, GloveObservations, GloveObservation, DepthObservation
from fabric_state_estimator.config import RuntimeSpec
from fabric_state_estimator.mesh import MaterialMesh, MeshSpec
from fabric_reference_reconstruction.bag import extract


def test_offline_import_uses_late_recorded_tf_and_raw_gloves():
    mesh = MaterialMesh(MeshSpec(rows=3, columns=3))
    runtime = RuntimeSpec(observation_topics=['/camera/fabric/observations'], robot_grip_frames=['blue_grip', 'silver_grip'])
    messages = []
    for i in range(2):
        header = Header(frame_id='camera'); header.stamp.sec = 1; header.stamp.nanosec = i*40_000_000
        info = CameraInfo(width=640, height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
        observation = FabricObservations(header=header, camera_id='camera', camera_info=info,
            depth_source='raw_registered_quality_gated', depth=[DepthObservation(u=320., v=240.,
            position=Point32(x=0., y=0., z=1.), axial_variance=.0001, quality=.5)])
        hands = GloveObservations(header=header, camera_id='camera', hands=[
            GloveObservation(hand_id=identity, position=Point(x=x, y=.84, z=1.), covariance=(np.eye(3)*.0001).ravel().tolist())
            for identity, x in [('human_left', 0.), ('human_right', .84)]])
        messages.extend([('/camera/fabric/observations', observation), ('/camera/fabric/gloves', hands)])
    transforms = []
    for child, x in [('camera', 0.), ('blue_grip', 0.), ('silver_grip', .84)]:
        tf = TransformStamped(header=Header(frame_id='world'), child_frame_id=child)
        tf.transform.rotation.w = 1.; tf.transform.translation.x = x
        if child != 'camera':
            tf.transform.translation.z = 1.
        transforms.append(tf)
    messages.append(('/tf_static', TFMessage(transforms=transforms)))  # arrives after images in bag
    frames, dropped = extract(messages, mesh, runtime)
    assert len(frames) == 2 and not dropped
    assert len(frames[0].grips) == 4
    targets, _ = frames[0].grip_targets()
    np.testing.assert_allclose(targets[:, 2], 1.)
    assert frames[0].views[0].time_ns == frames[0].time_ns


def test_mcap_reader_loads_sensor_contract_and_ignores_online_fabric_states(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from fabric_state_msgs.msg import FabricState
    from fabric_reference_reconstruction.bag import read_bag
    path = tmp_path/'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic, kind, message in [('/camera/fabric/observations', 'fabric_state_msgs/msg/FabricObservations', FabricObservations()),
                                 ('/fabric/estimated_state', 'fabric_state_msgs/msg/FabricState', FabricState())]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
        writer.write(topic, serialize_message(message), 1_000_000_000)
    del writer
    records = list(read_bag(path, 'mcap'))
    assert len(records) == 1 and records[0][0] == '/camera/fabric/observations'
