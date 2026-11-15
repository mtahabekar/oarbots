"""The simulator's sensitivity CBF, run on the twin's state.

`capsule_cbf.SensitivityCbfController` is used unchanged. It filters the two follower (robot)
grip velocities so the predicted sheet keeps its clearance from the obstacle's enclosing
capsule and stays inside the tension and strain limits, given the leader (human) grip
velocities. The controller works in the twin frame; this wrapper converts to and from the world.
"""
from dataclasses import replace
import numpy as np

from .vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.capsule import Capsule  # noqa: E402
from capsule_cbf.cloth_state import ClothState  # noqa: E402
from capsule_cbf.reduced_cloth import ReducedCloth  # noqa: E402
from capsule_cbf.sensitivity_cbf import SensitivityCbfController  # noqa: E402
from capsule_cbf.settings import Settings, load_settings  # noqa: E402,F401


class TwinCbf:
    def __init__(self, cloth, settings, period=4, max_speed=.1, margin=None):
        """`period`: twin frames per control update (the command is held in between).

        The controller is built for that control period with the same prediction model
        (horizon in seconds, output spacing, model step), as the simulator does for
        `--cbf-period`. `max_speed` caps the follower speed at what the arms can deliver.
        """
        cfg = settings
        stride = max(1, cfg.horizon_stride//period)
        if cfg.horizon_frames % period or (cfg.model_substeps*stride*period) % cfg.horizon_stride:
            raise ValueError(f'CBF period {period} does not divide the horizon ({cfg.horizon_frames} frames, stride {cfg.horizon_stride})')
        speed = min(cfg.max_speed, max_speed) if cfg.max_speed > 0 else max_speed
        self.settings = replace(cfg, horizon_stride=stride, horizon_frames=cfg.horizon_frames//period,
                                model_substeps=cfg.model_substeps*stride*period//cfg.horizon_stride, max_speed=speed,
                                margin=cfg.margin if margin is None else margin)
        self.cloth, self.period = cloth, period
        self.control_dt = period*cloth.spec.frame_dt
        center, half, _ = cloth.obstacle_twin
        self.capsule = Capsule(center, half, cfg.top_cap_mode, cfg.top_cap_radius_scale)
        self.rest_corners = cloth.targets.copy()
        self.build()

    def build(self):
        """(Re)build the reduced model and controller from the twin's current material.

        The reduced model copies stiffness from the Newton model, so this must run again after
        every material change. The formation reference and the last command are kept; the
        prediction-error history restarts, because it described the old model.
        """
        cloth, cfg = self.cloth, self.settings
        previous = getattr(self, 'controller', None)
        reduced = ReducedCloth(cloth.model, cloth.patch, cloth.patch_corners, cloth.patch_offsets, self.control_dt,
                               particle_start=0, particle_count=cloth.count, particle_radius=cloth.spec.particle_radius,
                               cells=cfg.prediction_grid_cells, substeps=cfg.model_substeps, stride=cfg.horizon_stride,
                               plant_substeps=cloth.spec.substeps*self.period, jacobian=cfg.model_jacobian)
        self.controller = SensitivityCbfController(self.capsule, reduced, self.rest_corners, cloth.spec.particle_radius,
                                                   cfg, self.control_dt)
        if previous is not None:
            self.controller.u_prev = previous.u_prev.copy()

    def update(self, leader_velocity, nominal=None):
        """Follower velocities (2, 3) in the world frame, and the controller's diagnostics.

        `leader_velocity` (2, 3): the human grip velocities, world. `nominal` (2, 3): an optional
        task command for the followers, world; the default follows the leaders and restores the
        initial follower-leader offsets.
        """
        cloth = self.cloth
        q, v = cloth.state_twin()
        leaders = cloth.vector_to_twin(leader_velocity)
        nominal = None if nominal is None else cloth.vector_to_twin(nominal)
        followers, debug = self.controller.update(ClothState(q, v), q[cloth.corner_particles], leaders, self.control_dt, nominal)
        followers = np.asarray(followers, float)
        speed = np.linalg.norm(followers, axis=1, keepdims=True)  # the controller bounds each axis; the arm limit is a norm
        followers = followers*np.minimum(1., self.settings.max_speed/np.maximum(speed, 1e-12))
        return cloth.vector_to_world(followers), debug

    def clearance(self):
        """Smallest signed distance (m) from the sheet to the capsule."""
        q, _ = self.cloth.state_twin()
        return float(self.capsule.distances(q, self.cloth.spec.particle_radius)[0].min())
