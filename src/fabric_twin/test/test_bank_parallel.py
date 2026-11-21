"""Parallel search semantics and pool failure/cleanup, without a CUDA device."""
import multiprocessing as mp
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('newton')
from fabric_twin.bank import Bank
from fabric_twin.bank_workers import CandidatePool, worker_count
from fabric_twin.cloth import TwinSpec
from test_bank_search import analytic_bank


@pytest.mark.parametrize('objective', [
    lambda x: .001**2 + .003**2*np.sum(np.log(x/[2., .5])**2),
    lambda x: .001,  # ties must keep the same winner
    lambda x: np.inf,
    lambda x: .01**2 - 1e-9*np.sum(np.log(x)),  # boundary optimum
])
@pytest.mark.parametrize('budget', [None, 1, 2, 6, 7])
def test_parallel_batches_reproduce_sequential_search(objective, budget):
    serial, evaluated = analytic_bank(objective)
    parallel, other_evaluated = analytic_bank(objective)
    batches, progress = [], []

    def costs(trials):
        batches.append(trials)
        return [parallel.cost(scales, [None]) for scales in trials]

    options = dict(rounds=8, max_evaluations=budget)
    expected = serial._calibrate([None], **options)
    actual = parallel._calibrate([None], **options, costs=costs, batch_size=3, progress=progress.append)
    assert actual == expected
    assert evaluated == other_evaluated
    assert all(len(batch) <= 3 for batch in batches)
    assert [p['evaluations'] for p in progress] == list(range(1, len(evaluated)+1))
    assert any(len(batch) > 1 for batch in batches) == (len(evaluated) > 2)


def test_time_cap_finishes_current_batch_then_stops(monkeypatch):
    import fabric_twin.bank as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'perf_counter', lambda: clock[0])
    bank, evaluated = analytic_bank(lambda x: .001+np.sum(np.log(x)**2))

    def costs(trials):
        clock[0] += .3  # concurrent simulations finish together
        return [bank.cost(scales, []) for scales in trials]

    result = bank._calibrate([], costs=costs, batch_size=3, max_seconds=.5)
    assert len(evaluated) == 4
    assert result['stop_reason'] == 'time_limit' and not result['converged']


def test_memory_cap_and_missing_mps_fall_back(monkeypatch):
    import builtins
    import io
    import fabric_twin.bank_workers as module
    monkeypatch.setattr(builtins, 'open', lambda *args: io.StringIO('MemAvailable: 17000000 kB\n'))
    monkeypatch.setattr(module.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(returncode=0))
    with pytest.warns(RuntimeWarning):
        assert worker_count(6, 'cuda:0') == 2
    monkeypatch.setattr(module.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(returncode=1))
    with pytest.warns(RuntimeWarning):
        assert worker_count(2, 'cuda:0') == 1
    assert worker_count(1, 'cuda:0') == 1


@pytest.mark.parametrize('workers', [0, -1, 1.5, True])
def test_invalid_worker_count(workers):
    with pytest.raises(ValueError):
        worker_count(workers, 'cuda:0')


def test_spawned_simulation_failure_reaches_parent_and_workers_are_joined():
    # Invalid geometry fails before CUDA initialization, inside the worker task.
    before = {child.pid for child in mp.active_children()}
    grips = np.array([[-.4, -.4, 1.], [.4, -.4, 1.], [-.4, .4, 1.], [.4, .4, 1.]])
    with pytest.raises(ValueError, match='Invalid twin specification'):
        with CandidatePool(2, TwinSpec(width_m=-1.), grips, []) as pool:
            pool.costs([(1., 1.), (2., 1.)])
    assert {child.pid for child in mp.active_children()} == before


def test_pool_is_joined_on_success_and_interrupt():
    before = {child.pid for child in mp.active_children()}
    with CandidatePool(2, None, None, []) as pool:
        assert pool.costs([]) == []
    with pytest.raises(KeyboardInterrupt):
        with CandidatePool(2, None, None, []):
            raise KeyboardInterrupt
    assert {child.pid for child in mp.active_children()} == before


def test_worker_death_does_not_leave_a_lost_task_waiting_forever():
    def lost_result(**kwargs):
        raise mp.TimeoutError

    pool = object.__new__(CandidatePool)
    pool.pool = SimpleNamespace(_pool=[SimpleNamespace(exitcode=-9)],
                                map_async=lambda *args, **kwargs: SimpleNamespace(get=lost_result))
    with pytest.raises(RuntimeError, match='worker exited'):
        pool.costs([(1., 1.)])
