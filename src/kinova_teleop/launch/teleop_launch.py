from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Requires a separately running command_guard and tablet mode/deadman heartbeats.
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace'),
        Node(package='kinova_teleop', executable='kinova_teleop', namespace=LaunchConfiguration('robot_namespace')),
    ])
