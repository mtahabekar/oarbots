"""Run on each computer a camera is plugged into (both helper laptops and the NUC).

    ros2 launch fabric_cell camera_host.launch.py camera:=blue_wrist [cell_dir:=/abs/path]

Starts `fabric_perception` for that camera next to its driver, so only compact observations and
glove measurements leave the computer. `cell_dir` holds this cell's `perception_<camera>.yaml`;
it defaults to the templates installed with this package, which must be tuned first.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERAS = {'blue_wrist': '/oarbot_blue/azure_kinect', 'silver_wrist': '/oarbot_silver/azure_kinect', 'overhead': '/nuc'}


def nodes(context):
    camera = LaunchConfiguration('camera').perform(context)
    if camera not in CAMERAS:
        raise RuntimeError(f'camera must be one of {sorted(CAMERAS)}')
    cell = LaunchConfiguration('cell_dir').perform(context) or str(Path(FindPackageShare('fabric_cell').perform(context))/'config')
    return [Node(package='fabric_perception', executable='camera', namespace=CAMERAS[camera], name='fabric_perception',
                 output='screen', parameters=[str(Path(cell)/f'perception_{camera}.yaml'), {'camera_id': camera}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', description='blue_wrist, silver_wrist or overhead'),
        DeclareLaunchArgument('cell_dir', default_value='', description='Directory with this cell\'s configuration files'),
        OpaqueFunction(function=nodes)])
