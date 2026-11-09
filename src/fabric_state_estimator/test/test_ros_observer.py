"""Synthetic observation-to-state ROS contract; no drivers or command topics."""
import time
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Point, Point32, WrenchStamped
from sensor_msgs.msg import CameraInfo
from fabric_state_msgs.msg import (FabricObservations, DepthObservation, HandStates, HandState,
                                   FabricModel, FabricState, FabricForecasts)
from fabric_state_estimator.node import FabricObserverNode


def spin(executor, duration):
    deadline = time.monotonic()+duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.005)


@pytest.mark.parametrize('mode', ['live', 'monitor', 'unsubscribed'])
def test_ros_model_state_forecast_and_stale_measurement_contract(tmp_path, mode):
    path = tmp_path/'config.yaml'
    path.write_text('mesh: {rows: 3, columns: 3}\nphysics: {backend: spring}\n'
        'observer: {ensemble_size: 8}\nruntime:\n  observation_topics: [/synthetic/fabric/observations]\n'
        '  forecast_horizons_sec: [0.33]\n  publish_rate_hz: 10.0\n')
    if mode == 'monitor':
        with path.open('a') as config:
            config.write('  publish_predictions: false\n  forecast_rate_hz: 0.0\n')
    rclpy.init(args=['--ros-args', '-p', 'config_file:='+str(path)])
    node = FabricObserverNode()
    rollout = node.observer.rollout
    horizons = []
    def record_rollout(values, *args, **kwargs):
        horizons.extend(values)
        return rollout(values, *args, **kwargs)
    node.observer.rollout = record_rollout
    producer = rclpy.create_node('observer_fixture', use_global_arguments=False)
    executor = SingleThreadedExecutor(); executor.add_node(node); executor.add_node(producer)
    try:
        models, states, forecasts = [], [], []
        producer.create_subscription(FabricModel, '/fabric/model', models.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        producer.create_subscription(FabricState, '/fabric/estimated_state', states.append, qos_profile_sensor_data)
        if mode != 'unsubscribed':
            producer.create_subscription(FabricForecasts, '/fabric/predicted_states', forecasts.append, qos_profile_sensor_data)
        hands_pub = producer.create_publisher(HandStates, '/fabric/hand_states', qos_profile_sensor_data)
        camera_pub = producer.create_publisher(FabricObservations, '/synthetic/fabric/observations', qos_profile_sensor_data)
        spin(executor, .15)
        capture = producer.get_clock().now().nanoseconds
        header = Header(stamp=Time(nanoseconds=capture).to_msg(), frame_id='camera_optical')
        def transform(child, translation, rotation):
            tf = TransformStamped(header=Header(frame_id='world'), child_frame_id=child)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, translation)
            q = Rotation.from_matrix(rotation).as_quat()
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
            node.tf.set_transform_static(tf, 'fixture')
        transform('camera_optical', [.42, .42, 2.], np.diag([1., -1., -1.]))
        transform('oarbot_blue/fabric_grip', [0., 0., 1.], np.eye(3))
        transform('oarbot_silver/fabric_grip', [.84, 0., 1.], np.eye(3))
        transform('force_sensor', [0., 0., 0.], Rotation.from_euler('z', np.pi/2).as_matrix())
        wrench = WrenchStamped(header=header)
        wrench.header = Header(stamp=Time(nanoseconds=capture-1_000_000).to_msg(), frame_id='force_sensor')
        wrench.wrench.force.x = 2.
        node.force_history[0].append(wrench)
        node.runtime.use_forces = True
        force = node.forces_at(capture)
        assert len(force) == 1
        np.testing.assert_allclose(force[0].on_sheet, [0., -2., 0.], atol=1e-12)
        assert node.forces_at(capture+200_000_000) == []  # do not refresh stale F/T
        node.runtime.use_forces = False
        covariance = np.diag([.002**2]*3+[.01**2]*3).ravel().tolist()
        # Fusion omits hands a camera group did not see: with unsynchronized
        # cameras each hand typically arrives in a different message.
        for offset, hand in ((3_000_000, HandState(hand_id='human_left', position=Point(x=0., y=.84, z=1.), covariance=covariance,
                                                    grip_uv=[0., 1.], holding_state=1, holding_is_assumed=True)),
                             (1_000_000, HandState(hand_id='human_right', position=Point(x=.84, y=.84, z=1.), covariance=covariance,
                                                    grip_uv=[1., 1.], holding_state=1, holding_is_assumed=True))):
            hands_pub.publish(HandStates(header=Header(stamp=Time(nanoseconds=capture-offset).to_msg(), frame_id='world'), hands=[hand]))
        spin(executor, .03)
        info = CameraInfo(header=header, width=640, height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
        msg = FabricObservations(header=header, camera_id='synthetic', camera_info=info,
            depth_source='raw_registered_quality_gated', depth=[DepthObservation(u=320., v=240.,
            position=Point32(x=0., y=0., z=1.1), axial_variance=.0001, quality=.5)])
        camera_pub.publish(msg); spin(executor, .14)
        assert models and list(models[-1].corner_indices) == [0, 2, 6, 8]
        assert states and all(s.measurement_stamp == header.stamp for s in states)
        assert len(states[-1].positions) == 9 and len(states[-1].covariance) == 9*36
        assert states[-1].model_id == models[-1].model_id
        from fabric_state_estimator.visualization import mesh_marker
        marker = mesh_marker(models[-1], states[-1])
        assert marker.points and len(marker.colors) == len(marker.points)
        assert marker.header == states[-1].header and marker.lifetime.nanosec == 500_000_000
        if mode == 'live':
            assert forecasts and forecasts[-1].states[0].measurement_stamp == header.stamp
            assert forecasts[-1].states[0].header.stamp != header.stamp
        else:
            assert not forecasts
        if mode == 'monitor':
            assert len(states) == 1 and not horizons
        elif mode == 'unsubscribed':
            assert max(horizons, default=0.) <= .2
        spin(executor, .15)
        count = len(states)
        spin(executor, .2)
        assert len(states) == count  # prediction deadline cannot make old data fresh
    finally:
        executor.shutdown(); node.destroy_node(); producer.destroy_node(); rclpy.shutdown()
