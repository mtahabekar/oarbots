"""Run on the compute machine (one NVIDIA GPU).

    ros2 launch fabric_cell compute.launch.py [cell_dir:=/abs/path] [twin:=true] [bank:=true]
        [commands:=false] [rviz:=true] [record:=/abs/dir] [nominal_grips:=false]

Starts the static calibration publisher, tagged-glove hand fusion, the estimator, and optionally
the fabric twin with its CBF, the parameter bank, RViz and the recorders. `commands:=false`
(default) is shadow mode: the twin computes robot commands and logs them, and sends zero.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    cell = Path(get('cell_dir') or Path(FindPackageShare('fabric_cell').perform(context))/'config')
    import yaml
    nominal = get('nominal_grips') == 'true'
    if nominal and get('commands') != 'false' and get('nominal_commands') != 'true':
        raise RuntimeError('Nominal grips are for shadow runs (commands:=false); add nominal_commands:=true to command '
                           'the robots on them in a supervised run')
    measured = yaml.safe_load((cell/'calibration.yaml').read_text()).get('measured', True)
    if not measured and not nominal:
        raise RuntimeError(f"{cell/'calibration.yaml'} still holds placeholder transforms (measured: false); "
                           'measure the cell calibration first')
    out = [LogInfo(msg=f'Using nominal grip transforms from {cell / "calibration.yaml"} (shadow run, no commands).')] if nominal else []
    out += [
        Node(package='fabric_sim_cell', executable='calibration_publisher', name='cell_calibration',
             parameters=[{'calibration_file': str(cell/'calibration.yaml'), 'errors_file': ''}]),
        Node(package='human_hand_tracking', executable='fusion', name='hand_fusion', output='screen',
             parameters=[str(cell/'fusion.yaml')]),
        Node(package='fabric_state_estimator', executable='observer', name='fabric_state_estimator', output='screen',
             parameters=[{'config_file': str(cell/'estimator.yaml')}]),
    ]
    if get('twin') == 'true':
        out.append(Node(package='fabric_twin', executable='twin', name='fabric_twin', output='screen',
                        # cell_dir/obstacle.yaml (a detected real obstacle, fabric_run.sh obstacle use) replaces
                        # twin.yaml's box when present
                        parameters=[{'config_file': str(cell/'twin.yaml'), 'commands_enabled': get('commands'),
                                     'obstacle_file': str(cell/'obstacle.yaml') if (cell/'obstacle.yaml').exists() else ''}]))
        if get('bank') == 'true':
            out.append(Node(package='fabric_twin', executable='parameter_bank', name='fabric_parameter_bank', output='screen',
                            parameters=[{'config_file': str(cell/'twin.yaml'), 'estimator_config': str(cell/'estimator.yaml'),
                                         'adapt_period_sec': float(get('adapt_period_sec'))}]))
    if get('rviz') == 'true':
        out.append(Node(package='fabric_sim_cell', executable='demo_markers', name='fabric_estimate_markers'))
        out.append(Node(package='rviz2', executable='rviz2', name='fabric_cell_rviz', output='log',
                        arguments=['-d', str(cell/'fabric_cell.rviz')]))
    if get('record'):
        record = Path(get('record'))
        out.append(Node(package='fabric_sim_cell', executable='recorder', name='estimate_recorder', output='screen',
                        parameters=[{'output': str(record/'estimate.npz')}]))
        if get('twin') == 'true':
            recorder = {'output': str(record/'twin.npz'), 'depth': get('record_depth') == 'true'}
            observer_settings = yaml.safe_load((cell/'estimator.yaml').read_text())
            observation_topics = observer_settings.get('runtime', {}).get('observation_topics')
            if observation_topics:
                recorder['depth_topics'] = observation_topics
            out.append(Node(package='fabric_twin', executable='twin_recorder', name='twin_recorder', output='screen',
                            parameters=[recorder]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_dir', default_value='', description='Directory with this cell\'s configuration files'),
        DeclareLaunchArgument('twin', default_value='true'),
        DeclareLaunchArgument('bank', default_value='true', description='Warm-up calibration and adaptation'),
        DeclareLaunchArgument('adapt_period_sec', default_value='10.0', description='Online adaptation period; 0 disables it'),
        DeclareLaunchArgument('commands', default_value='false', description="'true': send the twin's commands to the robots"),
        DeclareLaunchArgument('nominal_commands', default_value='false', choices=['false', 'true'],
                              description='Allow commands on nominal grips (supervised closed-loop test)'),
        DeclareLaunchArgument('nominal_grips', default_value='false', choices=['false', 'true'],
                              description='Allow nominal grip transforms in estimator-only shadow mode'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='', description='Directory for estimate.npz and twin.npz; empty = none'),
        DeclareLaunchArgument('record_depth', default_value='false', choices=['false', 'true'],
                              description='Save per-camera depth observations for replay'),
        OpaqueFunction(function=nodes)])
