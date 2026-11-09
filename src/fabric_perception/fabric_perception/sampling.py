"""Capture-clock sampling, independently of the glove detection cadence."""
import math


class FabricSampler:
    def __init__(self, rate_hz=0., monitor_rate_hz=1., adaptive=False, timeout_sec=2.5):
        if (not all(math.isfinite(v) for v in (rate_hz, monitor_rate_hz, timeout_sec))
                or rate_hz < 0 or monitor_rate_hz <= 0 or timeout_sec <= 0
                or adaptive and (rate_hz <= 0 or monitor_rate_hz > rate_hz)):
            raise ValueError('Invalid fabric sampling rates')
        self.rate, self.slow_rate, self.adaptive, self.timeout = rate_hz, monitor_rate_hz, adaptive, timeout_sec
        self.feedback_stamp = self.measurement_stamp = -1
        self.slow = False
        self.last, self.bucket, self.active_rate = None, None, None

    def feedback(self, stamp, measurement, slow):
        self.feedback_stamp, self.measurement_stamp, self.slow = stamp, measurement, slow

    def due(self, capture):
        fresh = (0 <= capture-self.feedback_stamp <= self.timeout*1e9
                 and 0 <= capture-self.measurement_stamp <= self.timeout*1e9)
        rate = self.slow_rate if self.adaptive and self.slow and fresh else self.rate
        if self.last is not None and capture < self.last:
            self.feedback_stamp = self.measurement_stamp = -1
            self.slow = False
            self.bucket = self.active_rate = None
            rate = self.rate
        bucket = int(math.floor(capture/1e9*rate+1e-7)) if rate else capture
        fire = rate != self.active_rate or bucket != self.bucket
        if self.active_rate is not None and 0 < rate < self.active_rate:
            # Join the common epoch instead of emitting a lone overhead check halfway
            # between wrist checks when feedback arrives during a capture period.
            phase = capture/1e9*rate
            fire = abs(phase-round(phase)) <= 1e-7
        self.last, self.bucket, self.active_rate = capture, bucket, rate
        return fire
