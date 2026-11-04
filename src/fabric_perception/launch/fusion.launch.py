"""Run once on the estimator host; TF must include calibrated wrist cameras."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('fabric_perception'), 'config', 'fusion.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='human_hand_tracking', executable='fusion', output='screen', parameters=[
            LaunchConfiguration('params_file'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
