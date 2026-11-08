"""Capture scheduling: whom the estimator waits for before assimilating a capture."""
from collections import deque
from time import monotonic
from types import SimpleNamespace

from fabric_state_estimator.config import RuntimeSpec
from fabric_state_estimator.node import FabricObserverNode

MS = 1_000_000


def node(intervals):
    now = monotonic()
    return SimpleNamespace(runtime=RuntimeSpec(), arrivals={'wrist': now, 'overhead': now},
                           intervals={k: deque(v, maxlen=10) for k, v in intervals.items()})


def test_slow_camera_is_waited_for_only_when_it_can_still_deliver_an_earlier_capture():
    wait = FabricObserverNode.may_still_deliver
    state = node({'overhead': [200 * MS] * 4})
    # The 5 Hz overhead delivered t=1.0 s: wrist captures at 1.067 and 1.133 s need not wait.
    assert not wait(state, 'overhead', 1000 * MS, 1067 * MS)
    assert not wait(state, 'overhead', 1000 * MS, 1133 * MS)
    # Its next capture (1.2 s) can still arrive: captures at or after it wait.
    assert wait(state, 'overhead', 1000 * MS, 1200 * MS)
    # Unknown period (few samples): wait, as before.
    assert wait(node({'overhead': [200 * MS]}), 'overhead', 1000 * MS, 1067 * MS)
    # A camera that already delivered this capture, or went silent, is never waited for.
    assert not wait(state, 'overhead', 1200 * MS, 1200 * MS)
    state.arrivals['overhead'] -= 10.
    assert not wait(state, 'overhead', 1000 * MS, 1200 * MS)
