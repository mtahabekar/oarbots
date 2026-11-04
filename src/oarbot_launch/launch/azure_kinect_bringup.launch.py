"""Typed Azure parameters. Requires the pinned driver plus repository patches."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PARAMETERS = {
    'depth_enabled': ('true', bool), 'depth_mode': ('NFOV_UNBINNED', str),
    'depth_unit': ('16UC1', str), 'color_enabled': ('true', bool),
    'color_format': ('bgra', str), 'color_resolution': ('1080P', str), 'fps': ('15', int),
    'point_cloud': ('false', bool), 'rgb_point_cloud': ('false', bool),
    'point_cloud_in_depth_frame': ('true', bool), 'sensor_sn': ('', str),
    'recording_file': ('', str), 'recording_loop_enabled': ('false', bool),
    'body_tracking_enabled': ('false', bool), 'body_tracking_smoothing_factor': ('0.0', float),
    'rescale_ir_to_mono8': ('false', bool), 'ir_mono8_scaling_factor': ('1.0', float),
    'imu_rate_target': ('100', int), 'wired_sync_mode': ('0', int),
    'subordinate_delay_off_master_usec': ('0', int), 'depth_delay_off_color_usec': ('0', int),
    'color_exposure_usec': ('0', int),
}


def validate(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    fps, mode = int(get('fps')), int(get('wired_sync_mode'))
    if fps not in (5, 15, 30) or mode not in (0, 1, 2):
        raise ValueError('Invalid FPS or wired sync mode')
    if fps == 30 and (get('depth_mode') == 'WFOV_UNBINNED' or get('color_resolution') == '3072P'):
        raise ValueError('WFOV_UNBINNED and 3072P do not support 30 FPS')
    if get('depth_unit') not in ('16UC1', '32FC1'):
        raise ValueError('depth_unit must be 16UC1 (mm) or 32FC1 (m)')
    period = 1_000_000 / fps
    if not 0 <= int(get('subordinate_delay_off_master_usec')) < period or abs(int(get('depth_delay_off_color_usec'))) >= period:
        raise ValueError('Camera offsets must be shorter than one frame period')
    if mode and (not get('sensor_sn') or int(get('color_exposure_usec')) <= 0 or get('color_enabled').lower() != 'true'):
        raise ValueError('Wired sync requires an explicit camera serial, fixed positive exposure and enabled color')
    if int(get('color_exposure_usec')) < 0:
        raise ValueError('color_exposure_usec is 0 for automatic exposure or a positive fixed exposure')
    prefix = get('frame_prefix')
    if prefix and (prefix.startswith('/') or not prefix.endswith('/')):
        raise ValueError('frame_prefix must be empty or have only a trailing slash')
    return []


def generate_launch_description():
    arguments = [DeclareLaunchArgument('azure_kinect_namespace'),
                 DeclareLaunchArgument('frame_prefix', default_value=''),
                 DeclareLaunchArgument('required', default_value='false')]
    arguments += [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in PARAMETERS.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in PARAMETERS.items()}
    parameters['tf_prefix'] = ParameterValue(LaunchConfiguration('frame_prefix'), value_type=str)
    node = Node(package='azure_kinect_ros_driver', executable='node', output='screen',
                namespace=LaunchConfiguration('azure_kinect_namespace'), parameters=[parameters])
    return LaunchDescription(arguments + [OpaqueFunction(function=validate),
        RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Required Azure camera exited'))]),
            condition=IfCondition(LaunchConfiguration('required'))), node])
