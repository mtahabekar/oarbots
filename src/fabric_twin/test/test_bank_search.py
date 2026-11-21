"""Convergence and termination checks with analytic costs; no GPU, ROS graph or arm commands."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('newton')
from fabric_twin.bank import Bank  # noqa: E402


def analytic_bank(objective, tick=lambda: None):
    bank, evaluated = object.__new__(Bank), []

    def cost(scales, cases):
        evaluated.append(tuple(scales))
        tick()
        return objective(np.array(scales)), [np.array([[np.log(scales[0]), np.log(scales[1]), 0.]])]

    bank.cost = cost
    return bank, evaluated


def test_search_stops_on_verified_convergence_before_its_round_budget():
    target = np.array([2., .5])
    bank, evaluated = analytic_bank(lambda x: .001**2 + .003**2*np.sum(np.log(x/target)**2))
    progress = []
    fit = bank.calibrate([None], rounds=8, progress=progress.append)
    assert fit['converged'] and fit['stop_reason'] == 'converged'
    assert fit['rounds_completed'] == 2
    assert fit['evaluations'] == len(evaluated) == len(progress) < 50
    np.testing.assert_allclose([fit['bend'], fit['shear']], target)
    # Both axes are checked at the declared 10% parameter tolerance before stopping.
    assert any(np.allclose(x, [2.2, .5]) for x in evaluated)
    assert any(np.allclose(x, [2., .55]) for x in evaluated)


def test_tiny_improvements_do_not_claim_convergence_while_parameters_move_to_an_edge():
    bank, _ = analytic_bank(lambda x: .01**2 - 1e-9*np.sum(np.log(x)))
    fit = bank.calibrate([None], span=4., rounds=4)
    assert not fit['converged']
    assert fit['stop_reason'] in ('parameter_bound', 'search_edge')


def test_evaluation_budget_is_not_reported_as_convergence():
    bank, evaluated = analytic_bank(lambda x: .001 + np.sum(np.log(x/[2., .5])**2))
    fit = bank.calibrate([None], rounds=8, max_evaluations=6)
    assert len(evaluated) == 6
    assert not fit['converged'] and fit['stop_reason'] == 'evaluation_limit'


def test_time_budget_is_not_a_minimum_duration(monkeypatch):
    import fabric_twin.bank as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'perf_counter', lambda: clock[0])
    bank, evaluated = analytic_bank(lambda x: .001 + np.sum(np.log(x/[2., .5])**2),
                                    tick=lambda: clock.__setitem__(0, clock[0]+.3))
    fit = bank.calibrate([None], rounds=8, max_seconds=.5)
    assert len(evaluated) == 2
    assert not fit['converged'] and fit['stop_reason'] == 'time_limit'


def test_invalid_surface_fits_stop_without_releasing_a_spurious_minimum():
    bank, _ = analytic_bank(lambda x: np.inf)
    fit = bank.calibrate([None], rounds=4)
    assert not fit['converged'] and fit['stop_reason'] == 'no_valid_surface_fit'
    assert fit['sensitivity_m'] == 0


@pytest.mark.parametrize(('converged', 'sensitivity', 'expected_release'), [
    (False, .01, False), (True, 0., False), (True, .01, True),
])
def test_bank_releases_only_converged_informative_results(converged, sensitivity, expected_release):
    pytest.importorskip('rclpy')
    from fabric_twin.bank_node import ParameterBankNode
    statuses, releases = [], []

    def calibrate(*args, **kwargs):
        assert kwargs['cost_tolerance_m'] == .0001
        assert kwargs['parameter_tolerance'] == .1
        assert kwargs['workers'] == 1
        kwargs['progress']({'evaluations': 1})
        return dict(bend=2., shear=.5, cost_m=.001, start_cost_m=.003, sensitivity_m=sensitivity,
                    evaluations=35, table=[], converged=converged,
                    stop_reason='converged' if converged else 'search_edge', rounds_completed=2, reach=5.65)

    node = SimpleNamespace(ensure_bank=lambda: True, case=lambda t: object(), bank=SimpleNamespace(calibrate=calibrate),
                           status=lambda message, **details: statuses.append((message, details)), scales=(1., 1.),
                           span=4., rounds=4, cost_tolerance=.0001, parameter_tolerance=.1,
                           max_search_seconds=600., max_evaluations=80, min_sensitivity=.0015,
                           search_workers=1,
                           release=lambda *args, **kwargs: releases.append((args, kwargs)),
                           get_logger=lambda: SimpleNamespace(warn=lambda text: None))
    ParameterBankNode.calibrate(node, dict(start_ns=0, holds=[(0, 3, 0), (4, 9, .02)]))
    assert bool(releases) is expected_release
    assert statuses[-1][0] == ('calibrated' if converged else 'calibration stopped')
    assert statuses[-1][1]['result']['release_expected'] is expected_release


def test_failed_worker_does_not_publish_a_release():
    pytest.importorskip('rclpy')
    from fabric_twin.bank_node import ParameterBankNode
    statuses, releases = [], []

    def fail(*args, **kwargs):
        raise RuntimeError('CUDA worker failed')

    node = SimpleNamespace(ensure_bank=lambda: True, case=lambda t: object(), bank=SimpleNamespace(calibrate=fail),
                           status=lambda message, **details: statuses.append(message), scales=(1., 1.),
                           span=4., rounds=4, cost_tolerance=.0001, parameter_tolerance=.1,
                           max_search_seconds=600., max_evaluations=80, search_workers=2,
                           release=lambda *args, **kwargs: releases.append(args),
                           get_logger=lambda: SimpleNamespace(error=lambda text: None))
    ParameterBankNode.calibrate(node, dict(start_ns=0, holds=[(0, 3, 0), (4, 9, .02)]))
    assert not releases
    assert statuses[-1].startswith('calibration skipped: simulation failed:')
