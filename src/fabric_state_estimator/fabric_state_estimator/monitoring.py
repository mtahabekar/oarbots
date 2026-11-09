"""Evidence-driven sampling policy; agreement is a diagnostic, not physical validation."""
from dataclasses import dataclass
import math


@dataclass
class MonitorSpec:
    enabled: bool = False
    warmup_sec: float = 3.0
    good_updates: int = 10
    timeout_sec: float = 2.5
    max_innovation: float = 4.0
    max_depth_p95_m: float = .02
    min_depth_factors: int = 16
    min_cameras: int = 2


class MonitorPolicy:
    def __init__(self, spec):
        values = [spec.warmup_sec, spec.timeout_sec, spec.max_innovation, spec.max_depth_p95_m]
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or min(spec.good_updates, spec.min_depth_factors, spec.min_cameras) < 1):
            raise ValueError('Invalid monitoring thresholds')
        self.spec = spec
        self.reset()

    def reset(self):
        self.first_good = self.last = None
        self.good = 0
        self.trusted = False
        self.cameras = {}

    def update(self, stamp, status, cameras):
        if self.last is not None and (stamp <= self.last or stamp-self.last > self.spec.timeout_sec*1e9):
            self.reset()
        self.last = stamp
        for camera in cameras:
            self.cameras[camera] = stamp
        diverse = sum(0 <= stamp-t <= 1.25e9 for t in self.cameras.values()) >= self.spec.min_cameras
        healthy = (not status.get('degraded', True) and not status.get('reinitialized', True)
                   and diverse and status.get('depth_factors', 0) >= self.spec.min_depth_factors
                   and math.isfinite(status.get('marginal_innovation_score', math.inf))
                   and status['marginal_innovation_score'] <= self.spec.max_innovation
                   and math.isfinite(status.get('prior_depth_p95_m', math.inf))
                   and status['prior_depth_p95_m'] <= self.spec.max_depth_p95_m)
        if not healthy:
            self.first_good, self.good, self.trusted = None, 0, False
        else:
            if self.first_good is None:
                self.first_good = stamp
            self.good += 1
            self.trusted = self.good >= self.spec.good_updates and stamp-self.first_good >= self.spec.warmup_sec*1e9

    def mode(self, now):
        if self.last is not None and not 0 <= now-self.last <= self.spec.timeout_sec*1e9:
            self.reset()
        return 'monitor' if self.spec.enabled and self.trusted else 'warmup'
