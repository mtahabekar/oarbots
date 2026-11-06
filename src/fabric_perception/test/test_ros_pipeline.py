"""Synthetic DDS/TF integration; never launches a driver or sends commands."""
import time
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from fabric_state_msgs.msg import FabricObservations, GloveObservation, GloveObservations, HandStates
from fabric_perception.node import FabricCamera
from human_hand_tracking.node import HandFusion


def spin(executor, seconds, action=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if action:
            action()
        executor.spin_once(timeout_sec=.005)


@pytest.fixture
def graph():
    nodes = []
    executor = None
    def start(kind, args=()):
        nonlocal executor
        rclpy.init(args=['--ros-args', *args])
        executor = SingleThreadedExecutor()
        target = kind()
        producer = rclpy.create_node('perception_fixture', use_global_arguments=False)
        nodes.extend([target, producer])
        for node in nodes:
            executor.add_node(node)
        return target, producer, executor
    yield start
    if executor is not None:
        executor.shutdown()
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def camera_inputs(producer):
    prefix = '/camera_fixture'
    rgb = producer.create_publisher(Image, prefix+'/rgb/image_raw', qos_profile_sensor_data)
    depth = producer.create_publisher(Image, prefix+'/depth_to_rgb/image_raw', qos_profile_sensor_data)
    info = producer.create_publisher(CameraInfo, prefix+'/rgb/camera_info', qos_profile_sensor_data)
    mask = producer.create_publisher(Image, prefix+'/fabric/robot_mask', qos_profile_sensor_data)
    return rgb, depth, info, mask


def images(producer, depth_value=1000, stamp=None):
    header = Header(stamp=stamp or producer.get_clock().now().to_msg(), frame_id='camera_fixture_optical')
    bridge = CvBridge()
    image = np.full((120, 160, 3), 220, np.uint8); image[20:100, 20:140] = 40
    rgb = bridge.cv2_to_imgmsg(image, 'bgr8'); rgb.header = header
    depth = bridge.cv2_to_imgmsg(np.full((120, 160), depth_value, np.uint16), '16UC1'); depth.header = header
    info = CameraInfo(header=header, width=160, height=120, distortion_model='plumb_bob',
                      k=[200., 0., 80., 0., 200., 60., 0., 0., 1.], d=[0.]*5)
    return rgb, depth, info


def test_camera_source_stamps_sensor_qos_depth_dropout_and_missing_gloves(graph):
    camera, producer, executor = graph(FabricCamera, ['-r', '__ns:=/camera_fixture', '-p', 'camera_id:=fixture'])
    rgb_pub, depth_pub, info_pub, _ = camera_inputs(producer)
    output, gloves = [], []
    producer.create_subscription(FabricObservations, '/camera_fixture/fabric/observations', output.append, qos_profile_sensor_data)
    producer.create_subscription(GloveObservations, '/camera_fixture/fabric/gloves', gloves.append, qos_profile_sensor_data)
    spin(executor, .25)
    def send(depth_value):
        rgb, depth, info = images(producer, depth_value)
        info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
        spin(executor, .12)
        return rgb, depth
    rgb, depth = send(1000)
    assert output and output[-1].header == rgb.header and output[-1].depth_stamp == depth.header.stamp
    assert output[-1].camera_id == 'fixture' and output[-1].depth and output[-1].boundaries
    assert output[-1].uv_priors == [] and output[-1].uv_source == 'unavailable'
    assert gloves and gloves[-1].hands == [] and gloves[-1].header == rgb.header
    rgb, _ = send(0)
    assert output[-1].header == rgb.header and output[-1].boundaries and not output[-1].depth
    assert output[-1].valid_depth_fraction == 0
    count = len(output)
    # Duplicate captures cannot refresh an observation or a track.
    info_pub.publish(images(producer)[2]); rgb_pub.publish(rgb)
    old_depth = images(producer, 0, rgb.header.stamp)[1]; depth_pub.publish(old_depth)
    spin(executor, .1)
    assert len(output) == count
    stale = Time(nanoseconds=producer.get_clock().now().nanoseconds-1_000_000_000).to_msg()
    rgb, depth, info = images(producer, stamp=stale)
    info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
    spin(executor, .1)
    assert len(output) == count
    # Pair with an incompatible depth frame and verify no accidental projection.
    rgb, depth, info = images(producer); depth.header = Header(stamp=rgb.header.stamp, frame_id='unregistered_depth')
    info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
    spin(executor, .1)
    assert len(output) == count


def test_camera_requires_exact_capture_robot_mask_when_configured(graph):
    _, producer, executor = graph(FabricCamera, ['-r', '__ns:=/camera_fixture', '-p', 'require_robot_mask:=true'])
    rgb_pub, depth_pub, info_pub, mask_pub = camera_inputs(producer)
    output = []
    producer.create_subscription(FabricObservations, '/camera_fixture/fabric/observations', output.append, qos_profile_sensor_data)
    spin(executor, .25)
    rgb, depth, info = images(producer)
    info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
    spin(executor, .1)
    assert not output
    rgb, depth, info = images(producer)
    mask = CvBridge().cv2_to_imgmsg(np.full((120, 160), 255, np.uint8), 'mono8'); mask.header = rgb.header
    info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
    spin(executor, .03)  # derived masks normally arrive after the RGB/depth pair
    assert not output
    mask_pub.publish(mask)
    spin(executor, .12)
    assert output and not output[-1].depth and not output[-1].boundaries


def glove_message(producer, stamp=None, frame='wrist_optical'):
    return GloveObservations(header=Header(stamp=stamp or producer.get_clock().now().to_msg(), frame_id=frame),
        camera_id='fixture_wrist', hands=[GloveObservation(hand_id=hand, tag_id=tag,
            position=Point(x=.1, y=.2, z=.7), covariance=(np.eye(3)*.0001).ravel().tolist())
            for hand, tag in (('human_left', 20), ('human_right', 21))])


def set_tf(fusion, stamp, translation):
    tf = TransformStamped(header=Header(stamp=stamp, frame_id='world'), child_frame_id='wrist_optical')
    tf.transform.rotation.w = 1.; tf.transform.translation.x = translation
    fusion.tf.set_transform(tf, 'fixture')


def test_fusion_uses_capture_tf_assumes_holding_and_does_not_refresh_missing_data(graph):
    fusion, producer, executor = graph(HandFusion, ['-p', 'observation_topics:=[/gloves_fixture]'])
    pub = producer.create_publisher(GloveObservations, '/gloves_fixture', qos_profile_sensor_data)
    output = []
    producer.create_subscription(HandStates, '/fabric/hand_states', output.append, qos_profile_sensor_data)
    spin(executor, .25)
    capture = producer.get_clock().now().nanoseconds - 100_000_000
    msg = glove_message(producer, Time(nanoseconds=capture).to_msg())
    set_tf(fusion, msg.header.stamp, 1.)
    set_tf(fusion, Time(nanoseconds=capture+80_000_000).to_msg(), 3.)
    pub.publish(msg); spin(executor, .12)
    assert output and len(output[-1].hands) == 2
    result = output[-1]
    assert result.header.stamp == msg.header.stamp and result.header.frame_id == 'world'
    for hand in result.hands:
        assert hand.position.x == pytest.approx(1.1)  # latest TF would incorrectly yield 3.1
        assert hand.holding_is_assumed and hand.holding_state == hand.HOLDING
        assert hand.source_cameras == ['fixture_wrist'] and hand.source_stamps == [msg.header.stamp]
    pub.publish(msg); spin(executor, .1)
    assert len(output) == 1
    # Missing transforms remove the observation (after the bounded TF wait)
    # rather than using another time/frame.
    pub.publish(glove_message(producer, frame='unavailable_optical')); spin(executor, .1)
    assert len(output) == 1
    spin(executor, .35)
    assert len(output) == 2 and output[-1].hands == []
    count = len(output)
    spin(executor, .35)
    assert len(output) == count


def test_fusion_waits_for_capture_time_tf_that_arrives_after_the_observation(graph):
    # Wrist TF passes through ArUco base localization, so the sample after a
    # wrist capture typically arrives later than the glove observation.
    fusion, producer, executor = graph(HandFusion, ['-p', 'observation_topics:=[/gloves_fixture]'])
    pub = producer.create_publisher(GloveObservations, '/gloves_fixture', qos_profile_sensor_data)
    output = []
    producer.create_subscription(HandStates, '/fabric/hand_states', output.append, qos_profile_sensor_data)
    spin(executor, .25)
    capture = producer.get_clock().now().nanoseconds - 20_000_000
    set_tf(fusion, Time(nanoseconds=capture-100_000_000).to_msg(), 1.)
    pub.publish(glove_message(producer, Time(nanoseconds=capture).to_msg())); spin(executor, .1)
    assert not output
    set_tf(fusion, Time(nanoseconds=capture+50_000_000).to_msg(), 4.)
    spin(executor, .05)
    assert output and len(output[-1].hands) == 2
    for hand in output[-1].hands:
        assert hand.position.x == pytest.approx(3.1)  # interpolated at capture time


def test_fusion_keeps_earlier_capture_from_slower_camera(graph):
    fusion, producer, executor = graph(HandFusion, ['-p', 'observation_topics:=[/gloves_a,/gloves_b]'])
    pubs = {c: producer.create_publisher(GloveObservations, '/gloves_' + c, qos_profile_sensor_data) for c in 'ab'}
    output = []
    producer.create_subscription(HandStates, '/fabric/hand_states', output.append, qos_profile_sensor_data)
    for camera in 'ab':
        tf = TransformStamped(header=Header(frame_id='world'), child_frame_id=camera + '_optical')
        tf.transform.rotation.w = 1.
        fusion.tf.set_transform_static(tf, 'fixture')
    spin(executor, .25)
    def send(camera, ns):
        msg = glove_message(producer, Time(nanoseconds=ns).to_msg(), camera + '_optical')
        msg.camera_id = camera
        pubs[camera].publish(msg)
    now = producer.get_clock().now().nanoseconds
    # Make both cameras live; these are already past max_wait, so no waiting.
    send('a', now-400_000_000); send('b', now-370_000_000); spin(executor, .05)
    assert len(output) == 2
    # b captures later but arrives first; a's earlier capture must not be dropped.
    send('b', now-20_000_000); spin(executor, .06)
    assert len(output) == 2
    send('a', now-40_000_000); spin(executor, .4)
    assert [list(o.hands[0].source_cameras) for o in output[2:]] == [['a'], ['b']]
    assert 'dropped' not in fusion.failure


def test_camera_static_exclusion_mask_removes_own_gripper(graph, tmp_path):
    path = str(tmp_path / 'gripper.png')
    mask = np.zeros((120, 160), np.uint8); mask[:, 80:] = 255
    cv2.imwrite(path, mask)
    _, producer, executor = graph(FabricCamera, ['-r', '__ns:=/camera_fixture', '-p', 'static_exclusion_mask:=' + path])
    rgb_pub, depth_pub, info_pub, _ = camera_inputs(producer)
    output = []
    producer.create_subscription(FabricObservations, '/camera_fixture/fabric/observations', output.append, qos_profile_sensor_data)
    spin(executor, .25)
    rgb, depth, info = images(producer)
    info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
    spin(executor, .15)
    assert output and output[-1].depth
    assert all(sample.u < 76 for sample in output[-1].depth)
    for boundary in output[-1].boundaries:
        for pixel, weight in zip(boundary.pixels, boundary.confidence):
            assert pixel.x < 77 and (pixel.x < 73 or weight == 0)


def test_fusion_rejects_duplicate_identity_atomically(graph):
    fusion, producer, executor = graph(HandFusion, ['-p', 'observation_topics:=[/gloves_fixture]'])
    pub = producer.create_publisher(GloveObservations, '/gloves_fixture', qos_profile_sensor_data)
    output = []
    producer.create_subscription(HandStates, '/fabric/hand_states', output.append, qos_profile_sensor_data)
    spin(executor, .25)
    msg = glove_message(producer)
    set_tf(fusion, msg.header.stamp, 0.)
    msg.hands.append(msg.hands[0])
    pub.publish(msg); spin(executor, .1)
    assert output and output[-1].hands == []


def test_rendered_rgbd_to_world_grip_pipeline(graph):
    camera, producer, executor = graph(FabricCamera, ['-r', '__ns:=/camera_fixture',
        '-p', 'observation_topics:=[/camera_fixture/fabric/gloves]'])
    fusion = HandFusion()
    executor.add_node(fusion)
    try:
        rgb_pub, depth_pub, info_pub, _ = camera_inputs(producer)
        views, world = [], []
        producer.create_subscription(GloveObservations, '/camera_fixture/fabric/gloves', views.append, qos_profile_sensor_data)
        producer.create_subscription(HandStates, '/fabric/hand_states', world.append, qos_profile_sensor_data)
        spin(executor, .25)
        image = np.full((480, 640, 3), (0, 220, 0), np.uint8)
        image[320:440, 80:560] = 40
        for identity, left in ((20, 100), (21, 420)):
            image[164:276, left-16:left+96] = 255
            dictionary = camera.gloves.dictionary
            marker = (cv2.aruco.drawMarker(dictionary, identity, 80) if hasattr(cv2.aruco, 'drawMarker')
                      else cv2.aruco.generateImageMarker(dictionary, identity, 80))
            image[180:260, left:left+80] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        header = Header(stamp=producer.get_clock().now().to_msg(), frame_id='camera_fixture_optical')
        transform = TransformStamped(header=Header(stamp=header.stamp, frame_id='world'),
                                     child_frame_id=header.frame_id)
        transform.transform.translation.x = .5; transform.transform.rotation.w = 1.
        fusion.tf.set_transform_static(transform, 'fixture')
        bridge = CvBridge()
        rgb = bridge.cv2_to_imgmsg(image, 'bgr8'); rgb.header = header
        depth = bridge.cv2_to_imgmsg(np.full((480, 640), 1000, np.uint16), '16UC1'); depth.header = header
        info = CameraInfo(header=header, width=640, height=480, distortion_model='plumb_bob',
                          k=[700., 0., 320., 0., 700., 240., 0., 0., 1.], d=[0.]*5)
        info_pub.publish(info); rgb_pub.publish(rgb); depth_pub.publish(depth)
        spin(executor, .18)
        assert views and len(views[-1].hands) == 2
        assert world and len(world[-1].hands) == 2
        by_hand = {hand.hand_id: hand for hand in views[-1].hands}
        for hand in world[-1].hands:
            assert hand.position.x == pytest.approx(by_hand[hand.hand_id].position.x + .5)
            assert hand.holding_is_assumed
            assert hand.grip_uv[1] == 1.
    finally:
        executor.remove_node(fusion)
        fusion.destroy_node()
