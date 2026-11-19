"""Overhead camera for a fabric session on the NUC, from the fabric workspace.

The stock NUC service (`nuc_launch.service`, ~/ros2_ws) runs the camera at 5 fps with ArUco base
localization on domain 0; its frames arrive about 1.15 s after capture (measured 2026-11-17). This
launch runs only the camera driver, at 15 fps, on the session domain, through the workspace's
`azure_kinect_bringup.launch.py` (launched by path: `oarbot_launch` is not built on the NUC because
colcon would require every robot driver it lists). Frames: `nuc/...`, topics `/nuc/...` as stock.

    fabric_session.sh start [fps:=15] [depth_mode:=NFOV_UNBINNED] [color_resolution:=1080P] [preview:=true]

preview:=true (default) also runs preview_relay.py: RViz on the laptop must show the 640 x 360 preview
(/nuc/preview/rgb/image_raw), never the full-resolution image, which saturates the NUC's link and
leaves the camera's frames 0.4-0.6 s old until it is restarted (see preview_relay.py).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BRINGUP = Path(__file__).resolve().parents[2] / 'src' / 'oarbot_launch' / 'launch' / 'azure_kinect_bringup.launch.py'
SETTINGS = {'fps': '15', 'depth_mode': 'NFOV_UNBINNED', 'color_resolution': '1080P', 'color_exposure_usec': '0',
            'sensor_sn': '', 'wired_sync_mode': '0', 'depth_delay_off_color_usec': '0'}


def generate_launch_description():
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in SETTINGS.items()]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(BRINGUP)),
        launch_arguments={'azure_kinect_namespace': 'nuc', 'frame_prefix': 'nuc/', 'required': 'true',
                          'point_cloud': 'false', 'rgb_point_cloud': 'false', 'body_tracking_enabled': 'false',
                          **{name: LaunchConfiguration(name) for name in SETTINGS}}.items())
    preview = ExecuteProcess(cmd=['python3', str(Path(__file__).resolve().parent / 'preview_relay.py')],
                             name='fabric_overhead_preview', output='screen',
                             condition=IfCondition(LaunchConfiguration('preview')))
    return LaunchDescription(arguments + [DeclareLaunchArgument('preview', default_value='true'), camera, preview])
