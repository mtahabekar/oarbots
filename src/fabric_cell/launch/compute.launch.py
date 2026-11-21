"""Run on the compute machine (one NVIDIA GPU).

    ros2 launch fabric_cell compute.launch.py [cell_dir:=/abs/path] [twin:=true] [bank:=true]
        [commands:=false] [rviz:=true] [record:=/abs/dir] [nominal_grips:=false]
        [start_bend:=1.0 start_shear:=1.0]

Starts the static calibration publisher, tagged-glove hand fusion, the estimator, and optionally
the fabric twin with its CBF, the parameter bank, RViz and the recorders. `commands:=false`
(default) is shadow mode: the twin computes robot commands and logs them, and sends zero.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
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
    # Starting material scales for both the twin and the bank (a warm-up test starts them deliberately wrong).
    start = {'start_bend': float(get('start_bend')), 'start_shear': float(get('start_shear'))}
    if get('twin') == 'true':
        out.append(Node(package='fabric_twin', executable='twin', name='fabric_twin', output='screen',
                        # cell_dir/obstacle.yaml (a detected real obstacle, fabric_run.sh obstacle use) replaces
                        # twin.yaml's box when present
                        parameters=[{'config_file': str(cell/'twin.yaml'), 'commands_enabled': get('commands'), **start,
                                     'obstacle_file': str(cell/'obstacle.yaml') if (cell/'obstacle.yaml').exists() else ''}]))
        if get('bank') == 'true':
            out.append(Node(package='fabric_twin', executable='parameter_bank', name='fabric_parameter_bank', output='screen',
                            parameters=[{'config_file': str(cell/'twin.yaml'), 'estimator_config': str(cell/'estimator.yaml'),
                                         'adapt_period_sec': float(get('adapt_period_sec')), **start,
                                         'search_span': float(get('search_span')), 'search_rounds': int(get('search_rounds')),
                                         'search_cost_tolerance_m': float(get('search_cost_tolerance_m')),
                                         'search_parameter_tolerance': float(get('search_parameter_tolerance')),
                                         'search_max_seconds': float(get('search_max_seconds')),
                                         'search_max_evaluations': int(get('search_max_evaluations')),
                                         'search_workers': int(get('search_workers'))}]))
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
        if get('record_bag') == 'true':
            # Everything else a report needs, raw: measured arm force/torque (the real sheet tension), joint states,
            # bridge/guard/camera diagnostics, deadman and e-stops, TF, gloves and hands, camera observations (depth),
            # the estimator's and twin's topics (state, status, CBF stream, corners), commands and arming.
            # Starts and stops with this launch (SIGINT closes the bag cleanly).
            out.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
                                           '-o', str(record/'bag'), '--regex', BAG_TOPICS, '--exclude-regex', BAG_EXCLUDE],
                                      name='run_bag', output='log'))
    return out


BAG_TOPICS = ('^/(fabric/.*|tf|tf_static|tablet_gui/deadman|nuc/fabric/(gloves|diagnostics|observations)'
              '|oarbot_(blue|silver)/(joint_states|force_torque_calibrated|force_torque_raw|diagnostics'
              '|fabric_cbf/.*|commands/fabric_cbf/.*|dingo/platform/emergency_stop'
              '|azure_kinect/fabric/(gloves|diagnostics|observations)))$')
BAG_EXCLUDE = '^/fabric/.*markers.*$'   # RViz drawings only


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_dir', default_value='', description='Directory with this cell\'s configuration files'),
        DeclareLaunchArgument('twin', default_value='true'),
        DeclareLaunchArgument('bank', default_value='true', description='Warm-up calibration and adaptation'),
        DeclareLaunchArgument('adapt_period_sec', default_value='10.0', description='Online adaptation period; 0 disables it'),
        DeclareLaunchArgument('start_bend', default_value='1.0', description="Twin and bank starting bend scale (of twin.yaml's)"),
        DeclareLaunchArgument('start_shear', default_value='1.0', description="Twin and bank starting shear scale (of twin.yaml's)"),
        DeclareLaunchArgument('search_span', default_value='4.0', description='Bank warm-up search: first-round factor'),
        DeclareLaunchArgument('search_rounds', default_value='3', description='Maximum bank search rounds (reach span^(1+1/4+1/8..))'),
        DeclareLaunchArgument('search_cost_tolerance_m', default_value='0.0001', description='Convergence RMS improvement tolerance'),
        DeclareLaunchArgument('search_parameter_tolerance', default_value='0.1', description='Relative coefficient stability tolerance'),
        DeclareLaunchArgument('search_max_seconds', default_value='600', description='Maximum fit time; not a minimum duration'),
        DeclareLaunchArgument('search_max_evaluations', default_value='80', description='Maximum candidate simulations'),
        DeclareLaunchArgument('search_workers', default_value='1', description='Independent bank GPU workers; >1 needs CUDA MPS; memory capped'),
        DeclareLaunchArgument('commands', default_value='false', description="'true': send the twin's commands to the robots"),
        DeclareLaunchArgument('nominal_commands', default_value='false', choices=['false', 'true'],
                              description='Allow commands on nominal grips (supervised closed-loop test)'),
        DeclareLaunchArgument('nominal_grips', default_value='false', choices=['false', 'true'],
                              description='Allow nominal grip transforms in estimator-only shadow mode'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='', description='Directory for estimate.npz and twin.npz; empty = none'),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['false', 'true'],
                              description='With record: also an mcap bag of forces, joints, diagnostics, TF, gloves, '
                                          'observations and the fabric topics (record/bag)'),
        DeclareLaunchArgument('record_depth', default_value='false', choices=['false', 'true'],
                              description='Save per-camera depth observations for replay'),
        OpaqueFunction(function=nodes)])
