"""Run next to one camera; its driver is launched independently."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_namespace', default_value='/oarbot_blue/azure_kinect'),
        DeclareLaunchArgument('camera_id', default_value='blue_wrist'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('fabric_perception'), 'config', 'camera.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='fabric_perception', executable='camera', namespace=LaunchConfiguration('camera_namespace'),
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'camera_id': ParameterValue(LaunchConfiguration('camera_id'), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
