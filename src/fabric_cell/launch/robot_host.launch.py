"""Run on each robot helper laptop, in addition to its normal launch.

    ros2 launch fabric_cell robot_host.launch.py robot:=blue [cell_dir:=/abs/path]

Starts that robot's wrist-camera perception and the fabric CBF bridge next to the command guard.
The bridge only produces arm commands after it is armed (`/<robot>/fabric_cbf/arm`), and the
guard only forwards them in control mode `fabric_cbf` with the deadman held.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('blue', 'silver'):
        raise RuntimeError("robot must be 'blue' or 'silver'")
    cell = Path(LaunchConfiguration('cell_dir').perform(context) or Path(FindPackageShare('fabric_cell').perform(context))/'config')
    bridge = yaml.safe_load((cell/'bridge.yaml').read_text())
    return [Node(package='fabric_perception', executable='camera', namespace=f'/oarbot_{robot}/azure_kinect',
                 name='fabric_perception', output='screen',
                 parameters=[str(cell/f'perception_{robot}_wrist.yaml'), {'camera_id': f'{robot}_wrist'}]),
            Node(package='oarbot_runtime', executable='fabric_cbf_bridge', namespace=f'oarbot_{robot}',
                 name='fabric_cbf_bridge', output='screen', parameters=[{**bridge['common'], **bridge.get(robot, {})}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='blue or silver'),
        DeclareLaunchArgument('cell_dir', default_value='', description='Directory with this cell\'s configuration files'),
        OpaqueFunction(function=nodes)])
