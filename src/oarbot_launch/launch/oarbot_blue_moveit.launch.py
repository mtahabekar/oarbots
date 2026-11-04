from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("oarbot_blue", package_name="oarbot_blue_moveit").to_moveit_configs()

    moveit_config.trajectory_execution["moveit_simple_controller_manager"]["gripper_controller"]["type"] = "GripperCommand"

    return LaunchDescription([
        DeclareLaunchArgument("allow_trajectory_execution", default_value="false"),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            namespace="oarbot_blue",
            parameters=[
                moveit_config.to_dict(),
                {"allow_trajectory_execution": ParameterValue(LaunchConfiguration("allow_trajectory_execution"), value_type=bool)}
            ],
            remappings=[
                (
                    "arm_controller/follow_joint_trajectory",
                    "kinova/j2n6s300/follow_joint_trajectory",
                ),
                (
                    "gripper_controller/gripper_command",
                    "kinova/j2n6s300_gripper/gripper_command",
                ),
            ],
        )
    ])