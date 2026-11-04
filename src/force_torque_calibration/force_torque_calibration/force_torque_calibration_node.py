import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
import csv
from kinova_msgs.action import ArmJointAngles
from kinova_msgs.msg import JointAngles
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from math import pi, isfinite
from time import monotonic
from pathlib import Path
from action_msgs.msg import GoalStatus
from kinova_msgs.srv import Stop
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

class ForceTorqueCalibration(Node):
    def __init__(self) -> None:
        super().__init__("force_torque_calibration")

        if not self.declare_parameter("allow_motion", False).value:
            raise RuntimeError("Calibration moves the arm. Review poses, stop command_guard and explicitly set allow_motion:=true in supervised setup mode")
        output = self.declare_parameter("output_file", "").value
        if output and Path(output).exists():
            raise FileExistsError("Choose a new calibration output file; existing calibration will not be replaced")
        self.motion_timeout = self.declare_parameter("motion_timeout_sec", 60.0).value
        self.collection_timeout = self.declare_parameter("collection_timeout_sec", 5.0).value
        if not all(isfinite(v) and v > 0 for v in (self.motion_timeout, self.collection_timeout)):
            raise ValueError("Timeouts must be finite and positive")
        self.stop_client = self.create_client(Stop, "kinova/j2n6s300_driver/in/stop")
        joint_angles_csv_description = ParameterDescriptor(description="Path to the csv file containing rows of 6 numbers, describing the joint angles of the Kinova arm")
        self.declare_parameter("joint_angles_csv", value="", descriptor=joint_angles_csv_description)
        joint_angles_csv_path = self.get_parameter("joint_angles_csv").get_parameter_value().string_value
        if joint_angles_csv_path == "":
            raise ValueError("joint_angles_csv is empty")

        tf_prefix_description = ParameterDescriptor(description="Prefix for tf frames; should include a trailing slash")
        self.declare_parameter("tf_prefix", value="", descriptor=tf_prefix_description)
        self.tf_prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value
        if self.tf_prefix and (self.tf_prefix.startswith('/') or not self.tf_prefix.endswith('/')):
            raise ValueError("tf_prefix must be empty or contain only a trailing slash")

        # Get the data from the CSV into a list
        self.joint_angles_list: list[JointAngles] = []
        with open(joint_angles_csv_path, newline="") as file:
            data = csv.reader(file)
            for row in data:
                # Make sure to convert from radians to degrees
                joint_angles_list = [float(angle) * 180 / pi for angle in row]
                if len(joint_angles_list) != 6 or not all(isfinite(v) for v in joint_angles_list):
                    raise ValueError("CSV rows must contain exactly six finite joint angles in radians")
                cur_joint_angles = JointAngles()
                cur_joint_angles.joint1 = joint_angles_list[0]
                cur_joint_angles.joint2 = joint_angles_list[1]
                cur_joint_angles.joint3 = joint_angles_list[2]
                cur_joint_angles.joint4 = joint_angles_list[3]
                cur_joint_angles.joint5 = joint_angles_list[4]
                cur_joint_angles.joint6 = joint_angles_list[5]
                self.joint_angles_list.append(cur_joint_angles)

        if not self.joint_angles_list:
            raise ValueError("Calibration pose file is empty")

        # Start the action client
        self.joint_angles_action = ActionClient(self, ArmJointAngles, "kinova/j2n6s300_driver/joint_angles")
        if not self.joint_angles_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Kinova joint angle action server is not available")
            raise RuntimeError("Kinova joint angle action server is not available")


        # Initialize the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def move_arm_to_state(self, joint_angles: JointAngles) -> None:
        goal_msg = ArmJointAngles.Goal()
        goal_msg.angles = joint_angles

        future = self.joint_angles_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.stop_arm()
            raise TimeoutError("Timed out waiting for motion goal acceptance")
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Calibration motion was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.motion_timeout)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.stop_arm()
            raise TimeoutError("Calibration motion timed out; stop requested")
        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.stop_arm()
            raise RuntimeError("Calibration motion failed")

    def stop_arm(self):
        if self.stop_client.wait_for_service(timeout_sec=1.0):
            future = self.stop_client.call_async(Stop.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.get_logger().error("Calibration aborted. Verify the arm is stopped before restarting the API")

    def settle(self, seconds):
        deadline = monotonic() + seconds
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline-monotonic())))

    def collect_force_torque_data(self) -> None:
        self.force_torque_data: list[WrenchStamped] = []
        self.force_torque_subscription = self.create_subscription(
            msg_type=WrenchStamped,
            topic="force_torque_raw",
            callback=self.handle_force_torque_data,
            qos_profile=qos_profile_sensor_data
        )

        # Collect data until we have enough (the subscription is stopped in the callback)
        deadline = monotonic() + self.collection_timeout
        try:
            while rclpy.ok() and len(self.force_torque_data) < 100:
                if monotonic() >= deadline:
                    raise TimeoutError("Not enough fresh force samples for calibration")
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self.stop_force_torque_subscription()

    def stop_force_torque_subscription(self) -> None:
        if hasattr(self, "force_torque_subscription"):
            self.destroy_subscription(self.force_torque_subscription)
            del self.force_torque_subscription

    def handle_force_torque_data(self, msg: WrenchStamped) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        values = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                  msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
        if not 0 <= age <= 0.2 or not all(isfinite(v) for v in values):
            return
        if self.force_torque_data and stamp <= Time.from_msg(self.force_torque_data[-1].header.stamp):
            return
        self.force_torque_data.append(msg)

        if len(self.force_torque_data) == 100:
            self.stop_force_torque_subscription()

    def get_gravity_vector(self) -> Vector3Stamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame=self.tf_prefix + "base_link",
                target_frame=self.tf_prefix + "j2n6s300_ft_measurement",
                time=Time()
            )

            gravity_vector_base_link = Vector3Stamped()
            gravity_vector_base_link.header.frame_id = self.tf_prefix + "base_link"
            gravity_vector_base_link.vector.x = 0.0
            gravity_vector_base_link.vector.y = 0.0
            gravity_vector_base_link.vector.z = -9.80665

            gravity_vector_force_torque = tf2_geometry_msgs.do_transform_vector3(gravity_vector_base_link, transform)

            return gravity_vector_force_torque



        except Exception as ex:
            self.get_logger().error(f"Could not get the transformation from {self.tf_prefix + "base_link"} to {self.tf_prefix + "j2n6s300_ft_measurement"}")
            raise LookupError("No gravity transform available") from ex

    