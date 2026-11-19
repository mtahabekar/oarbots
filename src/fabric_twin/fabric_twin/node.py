"""The fabric twin as a ROS node: no Isaac, no robot drivers.

Every frame the simulated sheet's four grip patches are put on the measured grips: the two
robot grips from TF and the two human grips from the tagged-glove hand states. The node runs a
fixed delay behind the clock, so those measurements are interpolated between samples instead
of extrapolated past the newest one. Each estimator analysis corrects the sheet with a bounded
force (`feedback.Nudger`). The sensitivity CBF runs on the twin's state against a simulated
obstacle and produces world-frame velocity commands for the two robot grips.

Commands are published for the robots only while enabled, and only while every input is fresh;
otherwise zero is published. The command guard and the bridge on each robot computer remain
the authority on whether an arm may move.
"""
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, TwistStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from fabric_state_msgs.msg import FabricModel, FabricState, HandStates
from human_hand_tracking.fusion import HandTrack

from .cbf import TwinCbf, default_settings_path, load_settings
from .cloth import Obstacle, TwinCloth, TwinSpec
from .feedback import FeedbackSpec, Nudger
from .vendor import fabric_sim_root
from .stretch import StretchGuard, StretchSpec
from .warmup import WarmupScript

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STATE_FIELDS = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'u', 'v')
ROBOTS = ('blue', 'silver')


def nanoseconds(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def load_twin_config(path):
    """Plain structured YAML: sections `sheet`, `obstacle`, `feedback`, `cbf`, `runtime`, `warmup`."""
    value = yaml.safe_load(Path(path).read_text()) or {}
    unknown = set(value)-{'sheet', 'obstacle', 'feedback', 'cbf', 'runtime', 'warmup', 'stretch_guard'}
    if unknown:
        raise ValueError(f'Unknown twin configuration sections: {sorted(unknown)}')
    sheet = dict(value.get('sheet', {}))
    for key in ('stretch', 'bend'):
        if key in sheet:
            sheet[key] = tuple(sheet[key])
    obstacle = Obstacle(**{k: tuple(v) if isinstance(v, list) else v for k, v in value.get('obstacle', {}).items()})
    runtime = dict(world_frame='world', robot_grip_frames=['oarbot_blue/fabric_grip', 'oarbot_silver/fabric_grip'],
                   hand_topic='/fabric/hand_states', estimate_topic='/fabric/estimated_state', delay_sec=.15,
                   hand_coast_sec=1., hand_start_age_sec=.5, estimate_stale_sec=1., state_every_frames=4,
                   max_catch_up_frames=4, initial_sag_m=.03, commands_enabled=False, hand_floor_m=.01,
                   leader_speed_limit_mps=.5, leader_smoothing_sec=.1, collision_clearance_m=0.,
                   formation_slack_m=.04, formation_lateral_slack_m=.02, formation_gain_per_s=2.,
                   warmup_at_sec=-1.)
    unknown = set(value.get('runtime', {}))-set(runtime)
    if unknown:
        raise ValueError(f'Unknown twin runtime settings: {sorted(unknown)}')
    runtime.update(value.get('runtime', {}))
    cbf = dict(enabled=True, config='', period=4, max_speed_mps=.1, margin_m=None, controller='sensitivity')
    unknown = set(value.get('cbf', {}))-set(cbf)
    if unknown:
        raise ValueError(f'Unknown twin cbf settings: {sorted(unknown)}')
    cbf.update(value.get('cbf', {}))
    stretch = dict(value.get('stretch_guard', {}))
    forces = dict(topics=stretch.pop('force_topics', ['/oarbot_blue/force_torque_calibrated',
                                                       '/oarbot_silver/force_torque_calibrated']),
                  sign=float(stretch.pop('force_on_robot_sign', 1.)))
    if forces['sign'] not in (-1., 1.) or len(forces['topics']) != 2:
        raise ValueError('stretch.force_on_robot_sign must be +1 or -1, with two force topics')
    return (TwinSpec(obstacle=obstacle, **sheet), FeedbackSpec(**value.get('feedback', {})), cbf, runtime,
            dict(value.get('warmup', {})), StretchSpec(**stretch), forces)


class FabricTwinNode(Node):
    def __init__(self):
        super().__init__('fabric_twin')
        default = str(Path(get_package_share_directory('fabric_twin'))/'config'/'twin.yaml')
        (self.spec, self.feedback_spec, self.cbf_options, self.runtime, warmup, stretch,
         self.force_options) = load_twin_config(self.declare_parameter('config_file', default).value)
        self.guard = StretchGuard(self.spec.width_m, self.spec.height_m, stretch)
        self.forces = [None, None]  # newest cloth pull on each robot: (receipt monotonic s, world vector)
        if stretch.force_feedback:
            for k, topic in enumerate(self.force_options['topics']):
                self.create_subscription(WrenchStamped, topic, lambda m, k=k: self.wrench(k, m), qos_profile_sensor_data)
        r = self.runtime
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.hands = {}  # corner (2, 3) -> HandTrack
        self.model, self.estimate, self.estimate_received = None, None, None
        self.cloth = self.cbf = self.nudger = None
        self.twin_time, self.frame, self.history = None, 0, deque(maxlen=120)
        self.pending_estimates = deque(maxlen=60)
        self.last_analysis_ns, self.feedback_info = None, {}
        self.command = np.zeros((2, 3)); self.debug = {}
        self.leader_velocity = np.zeros((2, 3))
        # Launch override of the configured shadow/active mode ('' keeps the file's value).
        override = str(self.declare_parameter('commands_enabled', '').value).lower()
        self.commands_enabled = bool(r['commands_enabled']) if override == '' else override in ('true', '1')
        self.frame_ms, self.cbf_ms = deque(maxlen=600), deque(maxlen=150)
        self.skipped, self.stale_grips, self.replaced = 0, 0, 0
        self.fault, self.material, self.pending_material = 'waiting for grips', dict(version=0, bend=1., shear=1.), None
        self.warmup = WarmupScript(**warmup)
        self.state_pub = self.create_publisher(PointCloud2, '/fabric/twin/state', qos_profile_sensor_data)
        self.corner_pub = self.create_publisher(PointCloud2, '/fabric/twin/corners', 50)
        self.marker_pub = self.create_publisher(MarkerArray, '/fabric/twin/markers', 5)
        self.status_pub = self.create_publisher(DiagnosticArray, '/fabric/twin/status', 10)
        self.debug_pub = self.create_publisher(String, '/fabric/twin/cbf_debug', 10)
        self.window_pub = self.create_publisher(String, '/fabric/twin/warmup_window', 5)
        self.warmup_announced = True
        self.shadow_pubs = [self.create_publisher(TwistStamped, f'/fabric/twin/follower_cmd/{c}', 10) for c in ROBOTS]
        self.command_pubs = [self.create_publisher(TwistStamped, f'/oarbot_{c}/fabric_cbf/cmd_world', 10) for c in ROBOTS]
        self.create_subscription(HandStates, r['hand_topic'], self.hand_states, qos_profile_sensor_data)
        self.create_subscription(FabricModel, '/fabric/model', self.set_model, LATCHED)
        self.create_subscription(FabricState, r['estimate_topic'], self.estimated, qos_profile_sensor_data)
        self.create_subscription(Bool, '/fabric/twin/enable_commands', self.enable, 1)
        self.create_subscription(String, '/fabric/twin/warmup', self.start_warmup, 1)
        self.create_subscription(String, '/fabric/material_parameters', self.material_release, LATCHED)
        self.create_timer(self.spec.frame_dt/2, self.tick)
        self.create_timer(.5, self.diagnostics)
        self.get_logger().info(f'fabric twin waiting for four grips; cloth library at {fabric_sim_root()}')

    # ---------------------------------------------------------------- inputs
    def hand_states(self, msg):
        stamp = nanoseconds(msg.header.stamp)
        if msg.header.frame_id != self.runtime['world_frame'] or stamp <= 0:
            return
        for hand in msg.hands:
            if hand.holding_state != hand.HOLDING or len(hand.grip_uv) != 2:
                continue
            uv = np.round(hand.grip_uv).astype(int)
            if uv[1] != 1 or uv[0] not in (0, 1):
                continue  # robots hold v = 0; only the two human corners are tracked here
            corner = 2+int(uv[0])
            p, v, a = hand.position, hand.velocity, hand.acceleration
            if len(hand.filter_covariance) == 81:
                covariance = np.asarray(hand.filter_covariance).reshape(9, 9)
            else:
                covariance = np.eye(9); covariance[:6, :6] = np.asarray(hand.covariance).reshape(6, 6)
            try:
                self.hands.setdefault(corner, HandTrack(floor=self.runtime['hand_floor_m'])).add(
                    stamp, [p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z], covariance, hand.jerk_stddev or 3.)
            except ValueError:
                pass

    def set_model(self, msg):
        self.model = msg
        if self.cloth is not None and (self.nudger is None or self.nudger.lift.shape[1] != msg.rows*msg.columns):
            self.nudger = Nudger(self.cloth, msg.rows, msg.columns, self.feedback_spec)

    def enable(self, msg):
        if msg.data and self.cloth is not None:
            # Follow from where everyone is now: the formation captured at the twin's start may be minutes
            # old (S5, 2026-11-18: the robots then chased that offset at the speed cap).
            grips, _ = self.grips_at(self.get_clock().now().nanoseconds, starting=True)
            if not np.isfinite(grips).all():
                self.get_logger().error('robot commands NOT enabled: a grip is not measured now')
                return
            self.set_formation(grips)
        self.commands_enabled = bool(msg.data)
        self.get_logger().warn(f'robot commands {"ENABLED" if self.commands_enabled else "disabled"}')

    def start_warmup(self, msg):
        if self.cloth is None:
            return
        self.warmup.start(self.twin_time, self.cloth.corners(), msg.data)
        self.warmup_announced = False
        self.get_logger().info(f'warm-up excitation started: {self.warmup.describe()}')

    def material_release(self, msg):
        try:
            release = json.loads(msg.data)
            wanted = dict(version=int(release['version']), bend=float(release['bend']), shear=float(release['shear']))
        except (ValueError, KeyError, TypeError):
            self.get_logger().error('ignored a malformed material parameter release')
            return
        if wanted['version'] != self.material['version']:
            self.pending_material = wanted

    def estimated(self, msg):
        """Queue one estimator analysis. The twin runs behind the clock, so an analysis usually
        arrives before the twin has reached its measurement time; it is applied when it has."""
        if msg.predicted:
            return
        self.estimate, self.estimate_received = msg, self.get_clock().now().nanoseconds
        if self.model is not None and msg.model_id == self.model.model_id and self.feedback_spec.enabled:
            self.pending_estimates.append(msg)

    def apply_estimates(self):
        """Correct the twin with every queued analysis whose measurement time it has now reached."""
        if self.nudger is None and self.model is not None:
            self.nudger = Nudger(self.cloth, self.model.rows, self.model.columns, self.feedback_spec)
        while self.pending_estimates and nanoseconds(self.pending_estimates[0].measurement_stamp) <= self.twin_time:
            msg = self.pending_estimates.popleft()
            stamp = nanoseconds(msg.measurement_stamp)
            match = min(self.history, key=lambda entry: abs(entry[0]-stamp), default=None)
            if (self.nudger is None or match is None or abs(match[0]-stamp) > 1.5e9*self.spec.frame_dt
                    or (self.last_analysis_ns is not None and stamp <= self.last_analysis_ns)):
                continue  # no twin state for that time (older than the history, or before the start)
            q = np.array([[p.x, p.y, p.z] for p in msg.positions])
            v = np.array([[p.x, p.y, p.z] for p in msg.velocities])
            covariance = np.asarray(msg.covariance).reshape(-1, 6, 6)
            dt = 1/15 if self.last_analysis_ns is None else min(.5, max(1e-3, (stamp-self.last_analysis_ns)/1e9))
            self.last_analysis_ns = stamp
            force, self.feedback_info = self.nudger.force(match[1], match[2], q, v, covariance, np.asarray(msg.visibility), dt)
            if self.feedback_info['diverged'] and not msg.degraded:
                # Far apart: the corrective force is not meant to pull the sheet across the room.
                self.cloth.place_from_mesh(q, self.model.rows, self.model.columns, self.cloth.to_world(self.cloth.targets), v)
                self.nudger.reset(); self.cloth.set_external(np.zeros((self.cloth.count, 3)))
                self.history.clear(); self.replaced += 1
                continue
            self.cloth.set_external(force)

    # ---------------------------------------------------------------- grips
    def grips_at(self, time_ns, starting=False):
        """Four grip positions (world) and the two human grip velocities at `time_ns`, or None."""
        r = self.runtime
        grips, leaders = np.full((4, 3), np.nan), np.zeros((2, 3))
        for corner, frame in enumerate(r['robot_grip_frames']):
            for when in (Time(nanoseconds=time_ns), Time()):  # the twin's time, else the newest transform
                try:
                    t = self.tf.lookup_transform(r['world_frame'], frame, when).transform.translation
                    grips[corner] = [t.x, t.y, t.z]
                    break
                except TransformException:
                    continue
        for corner in (2, 3):
            track = self.hands.get(corner)
            grip = None if track is None else track.at(time_ns, r['hand_start_age_sec'] if starting else r['hand_coast_sec'])
            if grip is not None:
                grips[corner], leaders[corner-2] = grip['position'], grip['velocity']
        return grips, leaders

    def wrench(self, k, msg):
        """Calibrated wrist force, rotated into the world at its own stamp."""
        try:
            q = self.tf.lookup_transform(self.runtime['world_frame'], msg.header.frame_id,
                                         Time.from_msg(msg.header.stamp)).transform.rotation
        except TransformException:
            return
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        f = msg.wrench.force
        self.forces[k] = (time.monotonic(), self.force_options['sign']*(rotation @ [f.x, f.y, f.z]))

    def measured_now(self):
        """Newest robot grips (TF), hand grips and hand velocities (glove filter carried to now)."""
        r, now = self.runtime, self.get_clock().now().nanoseconds
        robots, hands, velocity = np.full((2, 3), np.nan), np.full((2, 3), np.nan), np.zeros((2, 3))
        for k, frame in enumerate(r['robot_grip_frames']):
            try:
                t = self.tf.lookup_transform(r['world_frame'], frame, Time()).transform.translation
                robots[k] = [t.x, t.y, t.z]
            except TransformException:
                pass
            track = self.hands.get(2+k)
            hand = None if track is None else track.at(now, r['hand_coast_sec'])
            if hand is not None:
                hands[k], velocity[k] = hand['position'], hand['velocity']
        return robots, hands, velocity

    def guarded(self, command):
        """The CBF command after the stretch guard (priority over clearance); zero without measurements."""
        robots, hands, velocity = self.measured_now()
        if not (np.isfinite(robots).all() and np.isfinite(hands).all()):
            return np.zeros((2, 3)), dict(status='missing grips')
        forces = [None if f is None or time.monotonic()-f[0] > self.guard.spec.force_max_age_sec else f[1]
                  for f in self.forces]
        return self.guard.filter(robots, hands, velocity, command, forces)

    def follow_command(self):
        """Nominal robot grip velocities: follow each hand at the formation offset.

        Uses the freshest measurements, not the delayed twin: the hand estimate carried to now
        and the newest robot grip TF. The CBF then filters this command.
        """
        r, now = self.runtime, self.get_clock().now().nanoseconds
        command = np.zeros((2, 3))
        for k, frame in enumerate(r['robot_grip_frames']):
            track = self.hands.get(2+k)
            hand = None if track is None else track.at(now, r['hand_coast_sec'])
            try:
                t = self.tf.lookup_transform(r['world_frame'], frame, Time()).transform.translation
            except TransformException:
                continue
            if hand is None:
                continue
            velocity = hand['velocity']
            speed = np.linalg.norm(velocity)
            velocity = velocity*min(1., r['leader_speed_limit_mps']/max(speed, 1e-12))
            command[k] = velocity+r['formation_gain_per_s']*(hand['position']+self.formation[k]-np.array([t.x, t.y, t.z]))
        return command

    def set_formation(self, grips):
        # Formation: each robot grip keeps its starting offset from its human grip, shortened by
        # `formation_slack_m`. A taut sheet turns every centimetre of following lag into tens of
        # newtons (the first closed-loop rehearsal stretched the true sheet by 17 mm); with a few
        # centimetres of slack the lag only changes the sag.
        offset = grips[:2]-grips[2:]
        horizontal = offset.copy(); horizontal[:, 2] = 0.
        length = np.linalg.norm(horizontal, axis=1, keepdims=True)
        self.formation = offset-self.runtime['formation_slack_m']*horizontal/np.maximum(length, 1e-9)
        # The same along the robots' own edge: each robot grip sits this much toward the other.
        across = grips[1]-grips[0]; across[2] = 0.
        across /= max(np.linalg.norm(across), 1e-9)
        self.formation[0] += self.runtime['formation_lateral_slack_m']*across
        self.formation[1] -= self.runtime['formation_lateral_slack_m']*across

    def start(self, time_ns):
        grips, _ = self.grips_at(time_ns, starting=True)
        if not np.isfinite(grips).all():
            missing = [name for name, ok in zip(('blue grip TF', 'silver grip TF', 'left hand', 'right hand'),
                                                np.isfinite(grips).all(axis=1)) if not ok]
            self.fault = 'waiting for '+', '.join(missing)
            return
        started = time.perf_counter()
        self.cloth = TwinCloth(self.spec, grips)
        estimate = self.estimate
        if (self.feedback_spec.enabled and estimate is not None and self.model is not None
                and estimate.model_id == self.model.model_id
                and abs(nanoseconds(estimate.measurement_stamp)-time_ns) < 1e9):
            self.cloth.place_from_mesh(np.array([[p.x, p.y, p.z] for p in estimate.positions]),
                                       self.model.rows, self.model.columns, grips)
            source = 'the estimated mesh'
        else:
            self.cloth.place(grips, self.runtime['initial_sag_m'])
            source = 'a surface between the grips'
        if self.model is not None:
            self.nudger = Nudger(self.cloth, self.model.rows, self.model.columns, self.feedback_spec)
        if self.cbf_options['enabled']:
            path = self.cbf_options['config'] or str(default_settings_path())
            settings = load_settings(path)
            settings.controller = self.cbf_options['controller']  # 'none': the nominal command, unfiltered
            self.cbf = TwinCbf(self.cloth, settings, self.cbf_options['period'],
                               self.cbf_options['max_speed_mps'], self.cbf_options['margin_m'])
        self.set_formation(grips)
        self.twin_time, self.fault = time_ns, ''
        self.get_logger().info(f'twin started from {source} in {time.perf_counter()-started:.1f} s: '
                               f'{self.cloth.count} particles, obstacle clearance {self.cloth.obstacle_clearance():.3f} m')

    # ---------------------------------------------------------------- the frame loop
    def tick(self):
        r, dt_ns = self.runtime, int(round(self.spec.frame_dt*1e9))
        target = self.get_clock().now().nanoseconds-int(r['delay_sec']*1e9)
        if target <= 0:
            return
        if self.cloth is None:
            self.start(target)
            return
        if self.pending_material is not None:
            self.apply_material()
        behind = (target-self.twin_time)//dt_ns
        if behind > 30:  # more than half a second behind (a stall, or a clock jump): do not replay it
            self.skipped += int(behind); self.twin_time = target-dt_ns; self.history.clear(); behind = 1
        for _ in range(int(min(behind, r['max_catch_up_frames']))):
            self.step(self.twin_time+dt_ns)

    def step(self, time_ns):
        started = time.perf_counter()
        r = self.runtime
        grips, leaders = self.grips_at(time_ns)
        held = self.cloth.to_world(self.cloth.targets)
        stale = ~np.isfinite(grips).all(axis=1)
        if stale.any():  # an unmeasured grip holds its place; commands stop below
            grips[stale] = held[stale]; self.stale_grips += 1
        grips = self.cloth.within_reach(grips)
        self.cloth.step(grips)
        self.twin_time, self.frame = time_ns, self.frame+1
        q, v = self.cloth.state()
        self.history.append((time_ns, q, v))
        self.apply_estimates()
        blend = min(1., self.spec.frame_dt/max(r['leader_smoothing_sec'], self.spec.frame_dt))
        speed = np.linalg.norm(leaders, axis=1, keepdims=True)
        leaders = leaders*np.minimum(1., r['leader_speed_limit_mps']/np.maximum(speed, 1e-12))
        self.leader_velocity += blend*(leaders-self.leader_velocity)
        clearance = self.cloth.obstacle_clearance()
        collision = clearance < r['collision_clearance_m']
        estimate_age = (None if self.estimate_received is None
                        else (self.get_clock().now().nanoseconds-self.estimate_received)/1e9)
        self.fault = ('virtual collision with the obstacle' if collision else
                      'a grip measurement is missing' if stale.any() else
                      'estimator output is stale' if estimate_age is None or estimate_age > r['estimate_stale_sec'] else '')
        if self.cbf is not None and self.frame % self.cbf_options['period'] == 0:
            nominal = self.warmup.command(time_ns, held[:2]) if self.warmup.active(time_ns) else self.follow_command()
            try:
                self.command, self.debug = self.cbf.update(self.leader_velocity, nominal)
                self.command, self.debug['stretch_guard'] = self.guarded(self.command)
                self.cbf.applied(self.command, self.leader_velocity)
                self.cbf_ms.append(self.debug.get('controller_ms', 0.))
                if self.debug.get('status') == 'solver_failure':
                    self.fault = self.fault or 'CBF solver failure'
            except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
                self.command, self.fault = np.zeros((2, 3)), f'CBF error: {error}'
            self.publish_debug(clearance)
        if 0 <= r['warmup_at_sec'] <= time_ns/1e9 and self.warmup.start_ns is None:
            self.start_warmup(String(data='scheduled'))  # rehearsal convenience: start at a set clock time
        if not self.warmup_announced and not self.warmup.active(time_ns):
            # The excitation is over: tell the parameter bank which stretches were holds.
            self.warmup_announced = True
            self.window_pub.publish(String(data=json.dumps(dict(start_ns=self.warmup.start_ns, holds=self.warmup.holds()))))
        self.publish_commands(time_ns)
        self.publish_corners(time_ns, grips)
        if self.frame % r['state_every_frames'] == 0:
            self.publish_state(time_ns, q, v)
            self.publish_markers(time_ns, q)
        self.frame_ms.append(1000*(time.perf_counter()-started))

    def apply_material(self):
        wanted, self.pending_material = self.pending_material, None
        started = time.perf_counter()
        self.cloth.set_material(wanted['bend'], wanted['shear'])
        if self.cbf is not None:
            self.cbf.build()  # the reduced model copied the old stiffness
        self.material = wanted
        self.get_logger().info(f'material release {wanted["version"]}: bend x{wanted["bend"]:.3f}, shear x{wanted["shear"]:.3f} '
                               f'applied in {1000*(time.perf_counter()-started):.0f} ms')

    # ---------------------------------------------------------------- outputs
    def publish_commands(self, time_ns):
        stamp = self.get_clock().now().to_msg()
        sent = self.command if (self.commands_enabled and not self.fault and self.cbf is not None) else np.zeros((2, 3))
        for publishers, values in ((self.shadow_pubs, self.command), (self.command_pubs, sent)):
            for publisher, value in zip(publishers, values):
                msg = TwistStamped(header=Header(stamp=stamp, frame_id=self.runtime['world_frame']))
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(x) for x in value)
                publisher.publish(msg)

    def cloud(self, time_ns, columns, names):
        data = np.ascontiguousarray(np.column_stack(columns), dtype=np.float32)
        fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1) for i, name in enumerate(names)]
        return PointCloud2(header=Header(stamp=Time(nanoseconds=time_ns).to_msg(), frame_id=self.runtime['world_frame']),
                           height=1, width=len(data), fields=fields, is_bigendian=False, point_step=4*len(names),
                           row_step=4*len(names)*len(data), data=data.tobytes(), is_dense=True)

    def publish_state(self, time_ns, q, v):
        if self.state_pub.get_subscription_count():
            self.state_pub.publish(self.cloud(time_ns, (q, v, self.cloth.uv), STATE_FIELDS))

    def publish_corners(self, time_ns, grips):
        """The four grip targets this frame used: the boundary the parameter bank replays."""
        if self.corner_pub.get_subscription_count():
            self.corner_pub.publish(self.cloud(time_ns, (grips,), ('x', 'y', 'z')))

    def publish_debug(self, clearance):
        if not self.debug_pub.get_subscription_count():
            return
        skip = ('surfaces', 'rows', 'rhs', 'candidate_indices')
        debug = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.debug.items() if k not in skip}
        debug.update(box_clearance=clearance, twin_time=self.twin_time/1e9, commands_enabled=self.commands_enabled,
                     fault=self.fault, leader_velocity=self.leader_velocity.tolist(), follower_command=self.command.tolist())
        self.debug_pub.publish(String(data=json.dumps(debug, default=float)))

    def publish_markers(self, time_ns, q):
        if not self.marker_pub.get_subscription_count():
            return
        header = Header(stamp=Time(nanoseconds=time_ns).to_msg(), frame_id=self.runtime['world_frame'])

        def marker(ident, kind, scale, color):
            m = Marker(header=header, ns='twin', id=ident, type=kind, action=Marker.ADD)
            m.pose.orientation.w = 1.
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            return m
        nx, ny = self.spec.cells_x+1, self.spec.cells_y+1
        grid = np.lexsort((self.cloth.uv[:, 0], self.cloth.uv[:, 1])).reshape(ny, nx)
        stride = max(1, nx//14)
        lines = []
        for row in grid[::stride]:
            lines += [(a, b) for a, b in zip(row[:-1], row[1:])]
        for column in grid.T[::stride]:
            lines += [(a, b) for a, b in zip(column[:-1], column[1:])]
        color = (1., .25, .2, .95) if self.fault else (1., .6, .1, .95)
        wire = marker(0, Marker.LINE_LIST, (.004, 0., 0.), color)
        wire.points = [Point(x=float(q[i, 0]), y=float(q[i, 1]), z=float(q[i, 2])) for pair in lines for i in pair]
        o = self.spec.obstacle
        box = marker(1, Marker.CUBE, tuple(2*float(h) for h in o.half_extents), (.6, .6, .65, .9))
        box.pose.position.x, box.pose.position.y, box.pose.position.z = (float(c) for c in o.center)
        box.pose.orientation.z, box.pose.orientation.w = float(np.sin(o.yaw/2)), float(np.cos(o.yaw/2))
        radius = float(np.hypot(o.half_extents[0], o.half_extents[1]))
        envelope = marker(2, Marker.CYLINDER, (2*radius, 2*radius, 2*float(o.half_extents[2])), (.2, .5, 1., .25))
        envelope.pose.position.x, envelope.pose.position.y, envelope.pose.position.z = (float(c) for c in o.center)
        corners = marker(3, Marker.SPHERE_LIST, (.03, .03, .03), (1., 1., 1., 1.))
        corners.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.cloth.corners()]
        self.marker_pub.publish(MarkerArray(markers=[wire, box, envelope, corners]))

    def diagnostics(self):
        values = dict(fault=self.fault, commands_enabled=self.commands_enabled, frame=self.frame,
                      material=json.dumps(self.material), skipped_frames=self.skipped, stale_grip_frames=self.stale_grips,
                      replaced=self.replaced, warmup=self.warmup.describe())
        if self.cloth is not None and self.frame_ms:
            lag = (self.get_clock().now().nanoseconds-self.twin_time)/1e9
            values.update(frame_ms_p50=float(np.percentile(self.frame_ms, 50)), frame_ms_p95=float(np.percentile(self.frame_ms, 95)),
                          lag_sec=lag, box_clearance_m=self.cloth.obstacle_clearance(), **self.feedback_info)
            if self.cbf is not None and self.cbf_ms:
                values.update(cbf_ms_p95=float(np.percentile(self.cbf_ms, 95)), cbf_status=self.debug.get('status', ''),
                              capsule_clearance_m=self.debug.get('min_clearance', float('nan')))
        self.status_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[DiagnosticStatus(
            level=DiagnosticStatus.WARN if self.fault else DiagnosticStatus.OK, name=self.get_fully_qualified_name(),
            hardware_id='fabric_twin', message=self.fault or 'twin following the measured grips',
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]))


def main(args=None):
    rclpy.init(args=args)
    node = FabricTwinNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
