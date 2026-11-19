"""The fabric CBF bridge alone, with the real cell's bridge.yaml (common section plus this robot's).

robot_host.launch.py starts wrist perception and the bridge together; use this when perception runs
separately (scripts/hw/live_perception.sh). Started by `fabric_session.sh start-cell-bridge <cell_dir>`.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('blue', 'silver'):
        raise RuntimeError("robot must be 'blue' or 'silver'")
    bridge = yaml.safe_load((Path(LaunchConfiguration('cell_dir').perform(context)) / 'bridge.yaml').read_text())
    return [Node(package='oarbot_runtime', executable='fabric_cbf_bridge', namespace=f'oarbot_{robot}',
                 name='fabric_cbf_bridge', output='screen', parameters=[{**bridge['common'], **bridge.get(robot, {})}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot'), DeclareLaunchArgument('cell_dir'), OpaqueFunction(function=nodes)])
