"""The fabric CBF bridge for executing the simulated cell's twin commands on this robot.

    ros2 launch scripts/hw/sim_bridge.launch.py robot:=silver params_file:=~/fabric_ws/run/sim_bridge.yaml

Publishes the sim layout's world -> base_link and end_effector -> fabric_grip (calibration_exact.yaml
of fabric_sim_cell) and starts the bridge with the parameters written by sim_bridge.py. Run it only in
a fabric session domain: these transforms stand in for the lab calibration.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    calib = yaml.safe_load((Path(get_package_share_directory('fabric_sim_cell')) / 'config' / 'calibration_exact.yaml').read_text())
    wanted = {(calib['world_frame'], f'oarbot_{robot}/base_link'),
              (f'oarbot_{robot}/j2n6s300_end_effector', f'oarbot_{robot}/fabric_grip')}
    out = []
    for t in calib['transforms']:
        if (t['parent'], t['child']) not in wanted:
            continue
        x, y, z = t['translation']
        qx, qy, qz, qw = t['rotation_xyzw']
        out.append(Node(package='tf2_ros', executable='static_transform_publisher',
                        name='sim_layout_' + t['child'].replace('/', '_'),
                        arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--qx', str(qx), '--qy', str(qy),
                                   '--qz', str(qz), '--qw', str(qw), '--frame-id', t['parent'], '--child-frame-id', t['child']]))
    if len(out) != 2:
        raise RuntimeError(f'Sim layout transforms for {robot} not found in calibration_exact.yaml')
    out.append(Node(package='oarbot_runtime', executable='fabric_cbf_bridge', name='fabric_cbf_bridge',
                    namespace=f'oarbot_{robot}', output='screen', parameters=[params]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='blue or silver'),
        DeclareLaunchArgument('params_file', description='bridge parameters written by sim_bridge.py'),
        OpaqueFunction(function=nodes)])
