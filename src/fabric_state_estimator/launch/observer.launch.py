from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('fabric_state_estimator'), 'config', 'estimator.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='fabric_state_estimator', executable='observer', output='screen', parameters=[{
            'config_file': ParameterValue(LaunchConfiguration('config_file'), value_type=str),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
