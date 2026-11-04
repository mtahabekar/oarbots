from rclpy.action import ActionClient
from rclpy.client import Future
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from kinova_msgs.msg import FingerPosition, PoseVelocity
from kinova_msgs.action import SetFingersPosition, ArmJointAngles
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from time import monotonic
from math import isfinite
from rclpy.clock import Clock, ClockType
from math import pi

class RosGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("ros_gui_node")
        # TODO: This node should have an add_listener(oarbot_namespace) function that will add listeners to all the topics it needs so it doesn't have to wait on receiving data

        self.get_logger().info("Started ROS2 Node")
        self.last_deadman = self.last_estop = self.last_spacenav = float('-inf')
        self.admittance_publishers = {}
        self.mode_publishers = {}
        self.base_enable_publishers = {}
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog, clock=self.steady_clock)
        self.oarbot_settings_dict: dict[str, OarbotSettings] = dict()
        self.finger_position_subscribers: dict[str, Subscription] = dict()
        self.finger_position_actions: dict[str, ActionClient] = dict()
        self.arm_joint_angle_actions: dict[str, ActionClient] = dict()
        self.arm_velocity_publishers: dict[str, Publisher] = dict()
        self.base_velocity_publishers: dict[str, Publisher] = dict()

        self.spacenav_subscriber = self.create_subscription(
            msg_type=Twist,
            topic="spacenav/twist",
            callback=self.spacenav_callback,
            qos_profile=5
        )
        self.deadman_pressed = False
        self.deadman_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="deadman",
            callback=self.deadman_callback,
            qos_profile=1
        )
        self.e_stop_pressed = True
        self.dingo_e_stop_publishers: dict[str, Publisher] = dict()
        self.e_stop_subscriber = self.create_subscription(
            msg_type=Bool,
            topic="e_stop",
            callback=self.e_stop_callback,
            qos_profile=1
        )


    def active_node_or_topic(self, topic_node_name: str) -> bool:
        """
        Returns if the given node or topic name is active. This is used in status_indicator.py to allow the user to pass in either a node or topic name
        
        Specifically, the dingo status indicator needs to be a topic name because of the ros domain bridge acting on its nodes
        """
        return self.is_node_active(topic_node_name) or self.topic_has_publisher(topic_node_name)

    def is_node_active(self, node_name: str) -> bool:
        return node_name in self.get_fully_qualified_node_names()

    def topic_has_publisher(self, topic_name: str) -> bool:
        return len(self.get_publishers_info_by_topic(topic_name=topic_name)) > 0

    def get_cur_finger_percent(self, oarbot_namespace: str) -> int:
        return self.oarbot_settings_dict[oarbot_namespace].finger_position_percent

    def add_oarbot(self, oarbot_name: str) -> None:
        self.oarbot_settings_dict[oarbot_name] = OarbotSettings()

        self.finger_position_subscribers[oarbot_name] = self.create_subscription(
            msg_type=FingerPosition,
            topic=oarbot_name + "/kinova/j2n6s300_driver/out/finger_position",
            callback=lambda msg : self.finger_position_callback(oarbot_name, msg),
            qos_profile=3
        )
        self.finger_position_actions[oarbot_name] = ActionClient(self, SetFingersPosition, oarbot_name + "/kinova/j2n6s300_driver/finger_positions")
        self.arm_joint_angle_actions[oarbot_name] = ActionClient(self, ArmJointAngles, oarbot_name + "/kinova/j2n6s300_driver/joint_angles")
        self.arm_velocity_publishers[oarbot_name] = self.create_publisher(
            msg_type=PoseVelocity,
            topic=oarbot_name + "/commands/teleop/arm",
            qos_profile=1
        )
        self.base_velocity_publishers[oarbot_name] = self.create_publisher(
            msg_type=TwistStamped,
            topic=oarbot_name + "/commands/teleop/base",
            qos_profile=10
        )
        self.dingo_e_stop_publishers[oarbot_name] = self.create_publisher(
            msg_type=Bool,
            topic=oarbot_name + "/dingo/platform/emergency_stop",
            qos_profile=10
        )

        self.mode_publishers[oarbot_name] = self.create_publisher(String, oarbot_name + "/control_mode", 1)
        self.base_enable_publishers[oarbot_name] = self.create_publisher(Bool, oarbot_name + "/base_enable", 1)
        self.admittance_publishers[oarbot_name] = {
            suffix: self.create_publisher(Bool, oarbot_name + "/admittance_" + suffix, 1)
            for suffix in ("enable", "translation_enable", "rotation_enable")
        }

    def permitted(self):
        now = monotonic()
        return (self.deadman_pressed and not self.e_stop_pressed and
                now - self.last_deadman <= 0.2 and now - self.last_estop <= 0.2)

    def watchdog(self):
        permitted = self.permitted()
        for name, settings in self.oarbot_settings_dict.items():
            mode = 'disabled'
            if permitted:
                if settings.admittance_control_enabled:
                    mode = 'admittance'
                elif settings.arm_enabled:
                    mode = 'teleop'
            self.mode_publishers[name].publish(String(data=mode))
            self.base_enable_publishers[name].publish(Bool(data=permitted and settings.base_enabled))
            pubs = self.admittance_publishers[name]
            pubs['enable'].publish(Bool(data=mode == 'admittance'))
            pubs['translation_enable'].publish(Bool(data=settings.translation_enabled))
            pubs['rotation_enable'].publish(Bool(data=settings.rotation_enabled))
            if not permitted or monotonic() - self.last_spacenav > 0.15:
                self.arm_velocity_publishers[name].publish(PoseVelocity())
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = name.strip("/") + "/base_link"
                self.base_velocity_publishers[name].publish(msg)

    def finger_position_callback(self, oarbot_name: str, msg) -> None:
        avg_finger_position = (msg.finger1 + msg.finger2 + msg.finger3) / 3

        # This formula comes from the kinova library; to learn more, search for the variable finger_conv_ratio_ in kinova_arm.cpp
        finger_position_angle_rad = avg_finger_position * (80.0 / 6800.0) * pi  / 180.0
        # These come from kinova_common.xacro
        min_angle_rad = 0.0
        max_angle_rad = 1.51

        # Sometimes the angle given is beyond the range; clamp it
        clamped_finger_positon_angle_rad = max(min_angle_rad, min(finger_position_angle_rad, max_angle_rad))

        finger_position_open_percent = int(100 - 100 * (clamped_finger_positon_angle_rad - min_angle_rad) / (max_angle_rad - min_angle_rad))

        self.oarbot_settings_dict[oarbot_name].finger_position_percent = finger_position_open_percent

    def set_finger_position(self, oarbot_name: str, finger_position_percent: int) -> None:
        self.get_logger().warn("Position actions are unavailable in guarded velocity mode; use supervised setup mode")
        return
    def finger_goal_done(self, future: Future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Action goal was rejected")

    def arm_home(self, oarbot_name: str) -> None:
        self.get_logger().warn("Homing is unavailable in guarded velocity mode; use supervised setup mode")

    def arm_home_goal_done(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Action goal was rejected")

    def spacenav_callback(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(isfinite(v) for v in values):
            self.last_spacenav = float('-inf')
            self.watchdog()
            return
        self.last_spacenav = monotonic()
        if self.permitted():
            for oarbot_namespace, oarbot in self.oarbot_settings_dict.items():
                if oarbot.arm_enabled and not self.oarbot_settings_dict[oarbot_namespace].admittance_control_enabled:
                    arm_msg = PoseVelocity()

                    if oarbot.translation_enabled:
                        arm_msg.twist_linear_x = msg.linear.x
                        arm_msg.twist_linear_y = msg.linear.y
                        arm_msg.twist_linear_z = msg.linear.z

                        # For some reason, oarbot_silver has x-y values flipped
                        if oarbot_namespace == "/oarbot_silver":
                            arm_msg.twist_linear_x *= -1
                            arm_msg.twist_linear_y *= -1
                    if oarbot.rotation_enabled:
                        # Here, we need to swap the x and y angular values; just how the kinova is set up, I guess...
                        arm_msg.twist_angular_x = msg.angular.y
                        arm_msg.twist_angular_y = msg.angular.x
                        arm_msg.twist_angular_z = msg.angular.z

                        # For some reason, oarbot_blue has x-y values flipped
                        if oarbot_namespace == "/oarbot_blue":
                            arm_msg.twist_angular_x *= -1
                            arm_msg.twist_angular_y *= -1
        
                    self.arm_velocity_publishers[oarbot_namespace].publish(arm_msg)

                if oarbot.base_enabled:
                    base_msg = TwistStamped()
                    base_msg.header.stamp = self.get_clock().now().to_msg()
                    base_msg.header.frame_id = oarbot_namespace.strip("/") + "/base_link"

                    if oarbot.translation_enabled:
                        base_msg.twist.linear = msg.linear
                    if oarbot.rotation_enabled:
                        base_msg.twist.angular = msg.angular

                    self.base_velocity_publishers[oarbot_namespace].publish(base_msg)

    def deadman_callback(self, msg: Bool) -> None:
        self.last_deadman = monotonic()
        self.deadman_pressed = msg.data
        self.watchdog()

    def e_stop_callback(self, msg: Bool) -> None:
        self.last_estop = monotonic()
        self.e_stop_pressed = msg.data
        for publisher in self.dingo_e_stop_publishers.values():
            publisher.publish(msg)
        self.watchdog()

    def enable_admittance_control(self, oarbot_namespace: str) -> None:
        self.oarbot_settings_dict[oarbot_namespace].admittance_control_enabled = True
        self.watchdog()

    def disable_admittance_control(self, oarbot_namespace: str) -> None:
        self.oarbot_settings_dict[oarbot_namespace].admittance_control_enabled = False
        self.watchdog()


class OarbotSettings():
    def __init__(self) -> None:
        self.translation_enabled = True
        self.rotation_enabled = True
        self.arm_enabled = False
        self.base_enabled = False
        self.body_joint_following_enabled = False
        self.admittance_control_enabled = False
        self.finger_position_percent = 0