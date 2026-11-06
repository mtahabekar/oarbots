"""Shared strict configuration, usable without a ROS runtime."""
from dataclasses import asdict, dataclass, field
from pathlib import Path
import yaml
from .mesh import MaterialMesh, MeshSpec
from .physics import PhysicsSpec
from .ensemble import ObserverSpec


@dataclass
class RuntimeSpec:
    world_frame: str = 'world'
    observation_topics: list = field(default_factory=lambda: ['/oarbot_blue/azure_kinect/fabric/observations',
        '/oarbot_silver/azure_kinect/fabric/observations', '/nuc/fabric/observations'])
    hand_topic: str = '/fabric/hand_states'
    robot_grip_frames: list = field(default_factory=lambda: ['oarbot_blue/fabric_grip', 'oarbot_silver/fabric_grip'])
    robot_position_std_m: float = .005
    glove_extrinsic_std_m: float = .01  # Tier A import; match hand fusion extrinsic_stddev_m
    hand_max_age_sec: float = .25
    max_capture_age_sec: float = .75
    max_wait_sec: float = .4
    reorder_wait_sec: float = .02
    camera_sync_tolerance_sec: float = .015
    prediction_limit_sec: float = .2
    publish_rate_hz: float = 15.
    # Each forecast set simulates max(horizon) seconds for the whole ensemble.
    forecast_rate_hz: float = 2.
    forecast_horizons_sec: list = field(default_factory=lambda: [.1, .2, .33])
    use_forces: bool = False
    force_topics: list = field(default_factory=lambda: ['/oarbot_blue/force_torque_calibrated', '/oarbot_silver/force_torque_calibrated'])
    force_on_sheet_sign: float = -1.
    force_stddev_n: float = .5
    force_max_age_sec: float = .1


def load_config(path):
    value = yaml.safe_load(Path(path).read_text())
    allowed = {'mesh', 'physics', 'observer', 'observations', 'runtime', 'reference'}
    if not isinstance(value, dict) or set(value)-allowed:
        raise ValueError('Unknown configuration sections')
    mesh = MaterialMesh(MeshSpec(**value.get('mesh', {})))
    physics = PhysicsSpec(**value.get('physics', {}))
    observer = ObserverSpec(**value.get('observer', {}))
    runtime = RuntimeSpec(**value.get('runtime', {}))
    return mesh, physics, observer, runtime, value.get('observations', {}), value.get('reference', {})
