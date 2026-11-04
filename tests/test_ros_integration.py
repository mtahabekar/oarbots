import os
import pty
import struct
import subprocess
import time
from pathlib import Path
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, WrenchStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from aruco_interfaces.msg import ArucoMarkers


@pytest.fixture
def node():
    rclpy.init()
    node = rclpy.create_node('audit_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def processes():
    active = []
    def start(package, params=(), namespace='/fixture'):
        executable = Path(get_package_prefix(package))/'lib'/package/package
        args = [str(executable), '--ros-args', '-r', '__ns:='+namespace]
        for parameter in params:
            args += ['-p', parameter]
        log = open('/tmp/oarbots-test-' + package + '.log', 'w')
        process = subprocess.Popen(args, stdout=log, stderr=log)
        active.append((process, log))
        return process
    yield start
    for process, log in active:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()


def spin(node, seconds, action=None):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if action:
            action()
        rclpy.spin_once(node, timeout_sec=.01)


def stamp(node):
    return node.get_clock().now().to_msg()


def test_joint_state_best_effort_and_timestamp(node, processes):
    process = processes('joint_state_renamer', ['joint_state_prefix:=fixture/', 'input_topic:=input', 'output_topic:=output'])
    messages=[]
    node.create_subscription(JointState, '/fixture/output', messages.append, 10)
    pub=node.create_publisher(JointState, '/fixture/input', qos_profile_sensor_data)
    msg=JointState(); msg.header.stamp=stamp(node); msg.name=['joint']; msg.position=[.4]; msg.velocity=[.2]
    spin(node, .5, lambda: pub.publish(msg))
    assert process.poll() is None and messages
    assert messages[-1].name == ['fixture/joint']
    assert messages[-1].header.stamp == msg.header.stamp
    assert list(messages[-1].velocity) == [.2]
    messages.clear(); msg.position=[.1,.2]
    spin(node, .3, lambda: pub.publish(msg))
    assert all(len(m.position) == 1 for m in messages)


def test_tf_static_late_joiner_receives_all_batches(node, processes):
    processes('tf_relay', ['robot_namespace:=fixture', 'tf_frame_prefix:=robot/'])
    qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(TFMessage, '/fixture/tf_static', qos)
    def transform(child):
        t=TransformStamped(); t.header.frame_id='base'; t.child_frame_id=child; t.transform.rotation.w=1.; return t
    spin(node, .5, lambda: pub.publish(TFMessage(transforms=[transform('a')])))
    spin(node, .3, lambda: pub.publish(TFMessage(transforms=[transform('b')])))
    received=[]
    node.create_subscription(TFMessage, '/tf_static', received.append, qos)
    spin(node, .5)
    assert received
    assert {'robot/a','robot/b'} <= {t.child_frame_id for t in received[-1].transforms}


def test_root_tf_namespace_is_rejected(processes):
    process=processes('tf_relay', ['robot_namespace:=/'])
    assert process.wait(timeout=3) != 0


def test_arduino_disconnect_publishes_stop(node, processes):
    master, slave=pty.openpty()
    try:
        processes('tablet_arduino_publisher', ['com_port:='+os.ttyname(slave),
            'arduino_deadman_switch_topic:=deadman', 'arduino_e_stop_topic:=e_stop', 'serial_timeout_sec:=0.1'])
        deadman=[]; estop=[]
        node.create_subscription(Bool, '/fixture/deadman', lambda m: deadman.append(m.data), 1)
        node.create_subscription(Bool, '/fixture/e_stop', lambda m: estop.append(m.data), 1)
        spin(node, .7, lambda: os.write(master,b'0'))
        assert deadman[-1] and not estop[-1]
        spin(node, .3)
        assert not deadman[-1] and estop[-1]
    finally:
        os.close(master); os.close(slave)


def packet(values=(1.,2.,3.,4.,5.,6.), status=0):
    payload=struct.pack('<H6fIf',status,*values,123,25.)
    crc=0xffff
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc=(crc>>1)^0x8408 if crc&1 else crc>>1
    return b'\xaa'+payload+struct.pack('<H',crc^0xffff)


def test_ft_never_republishes_stale_or_untransformed_sample(node, processes):
    master, slave=pty.openpty()
    try:
        process=processes('rokubi_force_torque_publisher', ['serial_port:='+os.ttyname(slave), 'tf_prefix:=fixture/', 'max_sample_age_sec:=0.1'])
        raw=[]; calibrated=[]
        node.create_subscription(WrenchStamped,'/fixture/force_torque_raw',raw.append,qos_profile_sensor_data)
        node.create_subscription(WrenchStamped,'/fixture/force_torque_calibrated',calibrated.append,qos_profile_sensor_data)
        spin(node,.5,lambda:os.write(master,packet()))
        assert raw and not calibrated and process.poll() is None
        tf=TransformStamped(); tf.header.frame_id='fixture/base_link'; tf.child_frame_id='fixture/j2n6s300_ft_measurement'; tf.transform.rotation.w=1.
        broadcaster=StaticTransformBroadcaster(node); broadcaster.sendTransform(tf)
        spin(node,.4,lambda:os.write(master,packet()))
        assert calibrated
        result=calibrated[-1]
        assert result.header.frame_id == 'fixture/j2n6s300_ft_measurement'
        assert (result.wrench.force.x,result.wrench.force.y,result.wrench.force.z)==(-1.,2.,-3.)
        assert result.header.stamp in [m.header.stamp for m in raw]
        spin(node,.2); count=len(calibrated)
        spin(node,.2)
        assert len(calibrated)==count
        spin(node,.2,lambda:os.write(master,packet(status=2)))
        assert len(calibrated)==count
    finally:
        os.close(master); os.close(slave)


def test_localization_preserves_optical_frame_and_capture_stamp(node, processes):
    process=processes('oarbot_pose_publisher', ['aruco_markers_topic:=/test_markers','aruco_tag_ids:=[4]', 'oarbot_namespaces:=[fixture/]'])
    broadcaster=StaticTransformBroadcaster(node)
    rigid=TransformStamped(); rigid.header.frame_id='fixture/base_link'; rigid.child_frame_id='fixture/robot_arm_riser_aruco_mount_attach_link'; rigid.transform.rotation.w=1.
    broadcaster.sendTransform(rigid)
    pub=node.create_publisher(ArucoMarkers,'/test_markers',qos_profile_sensor_data)
    messages=[]
    node.create_subscription(TFMessage,'/tf',messages.append,10)
    spin(node,.5)
    msg=ArucoMarkers(); msg.header.frame_id='custom_optical'; msg.header.stamp=stamp(node); msg.marker_ids=[4]
    pose=Pose(); pose.position.z=2.; pose.orientation.w=1.; msg.poses=[pose]
    spin(node,.15,lambda:pub.publish(msg))
    output=[t for m in messages for t in m.transforms if t.child_frame_id=='fixture/base_link']
    assert output and output[-1].header == msg.header
    messages.clear(); msg.poses=[]
    spin(node,.2,lambda:pub.publish(msg))
    assert process.poll() is None


def test_command_guard_stops_on_heartbeat_loss_and_requires_release(node):
    from oarbot_runtime.command_guard import CommandGuard
    from kinova_msgs.msg import PoseVelocity
    from std_msgs.msg import String
    # Start the installed executable with its own namespace; only localhost domain 197.
    executable=Path(get_package_prefix('oarbot_runtime'))/'lib/oarbot_runtime/command_guard'
    process=subprocess.Popen([str(executable),'--ros-args','-r','__ns:=/guard_fixture'],stdout=open('/tmp/oarbots-python-node.log','w'),stderr=subprocess.STDOUT)
    try:
        deadman=node.create_publisher(Bool,'/tablet_gui/deadman',1)
        estop=node.create_publisher(Bool,'/tablet_gui/e_stop',1)
        mode=node.create_publisher(String,'/guard_fixture/control_mode',1)
        cmd=node.create_publisher(PoseVelocity,'/guard_fixture/commands/teleop/arm',1)
        received=[]
        node.create_subscription(PoseVelocity,'/guard_fixture/kinova/j2n6s300_driver/in/cartesian_velocity',received.append,1)
        def send(pressed):
            estop.publish(Bool(data=False)); deadman.publish(Bool(data=pressed))
            mode.publish(String(data='teleop')); cmd.publish(PoseVelocity(twist_linear_x=1.0))
        spin(node,.7,lambda:send(False))
        spin(node,.3,lambda:send(True))
        assert received and received[-1].twist_linear_x == pytest.approx(.1)
        spin(node,.4)
        assert process.poll() is None, Path("/tmp/oarbots-python-node.log").read_text()
        assert received[-1].twist_linear_x == 0.
        spin(node,.3,lambda:send(True))
        assert received[-1].twist_linear_x == 0.
        spin(node,.15,lambda:send(False)); spin(node,.2,lambda:send(True))
        assert received[-1].twist_linear_x == pytest.approx(.1)
    finally:
        process.terminate(); process.wait(timeout=3)


def test_human_adapter_uses_marker_ids_and_does_not_repeat_missing_hands(node):
    from visualization_msgs.msg import MarkerArray, Marker
    from geometry_msgs.msg import PoseWithCovarianceStamped
    executable=Path(get_package_prefix('oarbot_runtime'))/'lib/oarbot_runtime/human_hand_measurements'
    process=subprocess.Popen([str(executable),'--ros-args','-r','__ns:=/hands_fixture','-p','body_id:=7'],stdout=open('/tmp/oarbots-python-node.log','w'),stderr=subprocess.STDOUT)
    try:
        pub=node.create_publisher(MarkerArray,'/hands_fixture/body_tracking_data',qos_profile_sensor_data)
        received=[]
        node.create_subscription(PoseWithCovarianceStamped,'/hands_fixture/human/left_hand/pose',received.append,qos_profile_sensor_data)
        marker=Marker(); marker.id=708; marker.header.frame_id='camera_depth'; marker.pose.position.x=.5
        def send():
            marker.header.stamp=stamp(node); pub.publish(MarkerArray(markers=[marker]))
        spin(node,.6,send)
        assert received and received[-1].pose.pose.position.x == .5
        assert received[-1].header.frame_id == 'camera_depth'
        assert received[-1].pose.covariance[0] == pytest.approx(.05**2)
        spin(node,.15); count=len(received)
        marker.id=808
        spin(node,.25,send)
        assert len(received)==count
    finally:
        process.terminate(); process.wait(timeout=3)


def test_admittance_stops_on_tf_failure_disable_and_sensor_loss(node, processes):
    from kinova_msgs.msg import PoseVelocity
    process=processes('admittance_control',['tf_prefix:=fixture/'])
    enable=node.create_publisher(Bool,'/fixture/admittance_enable',1)
    force=node.create_publisher(WrenchStamped,'/fixture/force_torque_calibrated',qos_profile_sensor_data)
    messages=[]
    node.create_subscription(PoseVelocity,'/fixture/commands/admittance/arm',messages.append,1)
    frame='fixture/j2n6s300_ft_measurement'
    def send(enabled=True, frame_id=frame):
        enable.publish(Bool(data=enabled))
        msg=WrenchStamped(); msg.header.stamp=stamp(node); msg.header.frame_id=frame_id
        msg.wrench.force.x=30.; force.publish(msg)
    spin(node,.4,send)
    assert messages and not any(m.twist_linear_x for m in messages)
    broadcaster=StaticTransformBroadcaster(node)
    transforms=[]
    for child in ('j2n6s300_ft_measurement','j2n6s300_ft_robot_side_connector'):
        tf=TransformStamped(); tf.header.frame_id='fixture/j2n6s300_link_base'
        tf.child_frame_id='fixture/'+child; tf.transform.rotation.w=1.; transforms.append(tf)
    broadcaster.sendTransform(transforms)
    spin(node,.4,send)
    assert messages[-1].twist_linear_x > 0.
    assert messages[-1].twist_linear_x <= .100001
    spin(node,.15,lambda:send(False))
    assert messages[-1].twist_linear_x == 0.
    spin(node,.3,send)
    spin(node,.2,lambda:enable.publish(Bool(data=True)))
    assert messages[-1].twist_linear_x == 0.
    spin(node,.2,lambda:send(True,'missing_sensor_frame'))
    assert messages[-1].twist_linear_x == 0. and process.poll() is None


def test_base_guard_rejects_wrong_frame_and_limits_planar_speed(node):
    from oarbot_runtime.command_guard import CommandGuard
    from geometry_msgs.msg import TwistStamped
    # Test the actual callback with fake monotonic receipt leases; no driver is launched.
    rclpy_node = CommandGuard.__new__(CommandGuard)
    from oarbot_runtime.guard import Guard
    rclpy_node.guard=Guard(); rclpy_node.limits=(.1,.3,.15,.3)
    rclpy_node.get_namespace=lambda:'/fixture'
    rclpy_node.get_clock=node.get_clock
    now=time.monotonic()
    rclpy_node.guard.estop_update(False,now)
    rclpy_node.guard.deadman_update(False,now)
    rclpy_node.guard.deadman_update(True,now)
    rclpy_node.guard.base_update(True,now)
    msg=TwistStamped(); msg.header.stamp=stamp(node); msg.header.frame_id='world'; msg.twist.linear.x=1.
    rclpy_node.base_command(msg)
    assert not any(rclpy_node.guard.output(time.monotonic())[1])
    msg.header.frame_id='fixture/base_link'
    rclpy_node.base_command(msg)
    assert rclpy_node.guard.output(time.monotonic())[1][0] == pytest.approx(.15)


def test_aruco_node_publishes_stamped_pose_on_stock_opencv(node):
    from sensor_msgs.msg import CameraInfo, Image
    from cv_bridge import CvBridge
    from test_aruco import marker_image
    executable=Path(get_package_prefix('aruco_pose_estimation'))/'lib/aruco_pose_estimation/aruco_node.py'
    arguments=[str(executable),'--ros-args']
    for value in ('use_depth_input:=false','marker_size:=0.2','aruco_dictionary_id:=DICT_4X4_50',
        'image_topic:=/aruco_fixture/image','camera_info_topic:=/aruco_fixture/info',
        'camera_frame:=test_optical','detected_markers_topic:=/aruco_fixture/markers'):
        arguments += ['-p',value]
    with open('/tmp/oarbots-aruco-node.log','w') as log:
        process=subprocess.Popen(arguments,stdout=log,stderr=log)
        try:
            info_pub=node.create_publisher(CameraInfo,'/aruco_fixture/info',qos_profile_sensor_data)
            image_pub=node.create_publisher(Image,'/aruco_fixture/image',qos_profile_sensor_data)
            received=[]
            node.create_subscription(ArucoMarkers,'/aruco_fixture/markers',received.append,10)
            info=CameraInfo(); info.width=640; info.height=480; info.header.frame_id='test_optical'
            info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]; info.d=[0.]*5
            image=CvBridge().cv2_to_imgmsg(marker_image(),encoding='rgb8')
            image.header.frame_id='test_optical'
            stamps=[]
            def send():
                image.header.stamp=stamp(node); stamps.append(image.header.stamp)
                info_pub.publish(info); image_pub.publish(image)
            deadline = time.monotonic()+5.
            while not received and process.poll() is None and time.monotonic() < deadline:
                send()
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() is None, Path('/tmp/oarbots-aruco-node.log').read_text()
            assert received and list(received[-1].marker_ids)==[4]
            assert received[-1].header.stamp in stamps
            assert received[-1].header.frame_id=='test_optical'
        finally:
            process.terminate(); process.wait(timeout=3)


def test_vendor_pose_demo_requires_opt_in_and_uses_ros2_action_result(node):
    from kinova_msgs.action import ArmPose
    from rclpy.action import ActionServer
    executable = Path(get_package_prefix('kinova_demo'))/'lib/kinova_demo/pose_action_client.py'
    refused = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert refused.returncode != 0 and 'execute_motion:=true' in refused.stderr
    goals = []

    def execute(handle):
        goals.append(handle.request)
        result = ArmPose.Result()
        result.pose = handle.request.pose
        handle.succeed()
        return result

    server = ActionServer(node, ArmPose, '/demo_fixture/j2n6s300_driver/tool_pose', execute)
    args = [str(executable), '--ros-args', '-r', '__ns:=/demo_fixture']
    for parameter in ('execute_motion:=true', 'position:=[0.1,0.2,0.3]',
                      'orientation_xyzw:=[0.0,0.0,0.0,2.0]',
                      'frame_id:=demo_fixture/base', 'timeout_sec:=2.0'):
        args += ['-p', parameter]
    with open('/tmp/oarbots-pose-demo.log', 'w') as log:
        process = subprocess.Popen(args, stdout=log, stderr=log)
        try:
            deadline = time.monotonic()+5.
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() == 0, Path('/tmp/oarbots-pose-demo.log').read_text()
            assert len(goals) == 1
            pose = goals[0].pose
            assert pose.header.frame_id == 'demo_fixture/base'
            assert pose.pose.position.x == pytest.approx(.1)
            assert pose.pose.orientation.w == pytest.approx(1.)
        finally:
            if process.poll() is None:
                process.terminate(); process.wait(timeout=3)
            server.destroy()
