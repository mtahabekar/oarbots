import pytest
from fabric_perception.sampling import FabricSampler


def test_glove_cadence_is_independent_of_fabric_sampling_and_feedback_expiry():
    sampler = FabricSampler(5., 1., True)
    # No feedback: all five 5 Hz captures are fabric checks.
    assert sum(sampler.due(k*200_000_000) for k in range(5)) == 5
    sampler.feedback(1_000_000_000, 1_000_000_000, True)
    assert sum(sampler.due(1_000_000_000+k*200_000_000) for k in range(5)) == 1
    # Fresh diagnostics with an old analysis cannot sustain low-rate mode.
    sampler.feedback(4_000_000_000, 1_000_000_000, True)
    assert sum(sampler.due(4_000_000_000+k*200_000_000) for k in range(5)) == 5


def test_failure_and_clock_reset_restore_high_rate_immediately():
    sampler = FabricSampler(5., 1., True)
    sampler.feedback(2_000_000_000, 2_000_000_000, True)
    assert sampler.due(2_000_000_000)
    assert not sampler.due(2_200_000_000)
    sampler.feedback(2_200_000_000, 2_000_000_000, False)
    assert sampler.due(2_200_000_000)
    sampler.feedback(2_400_000_000, 2_400_000_000, True)
    sampler.due(2_400_000_000)
    assert sampler.due(200_000_000)
    assert sampler.due(400_000_000)


def test_default_processes_every_image_and_invalid_rates_fail_early():
    sampler = FabricSampler()
    assert all(sampler.due(k*10_000_000) for k in range(50))
    for args in [(-1.,), (float('nan'),), (0., 1., True), (1., 5., True)]:
        with pytest.raises(ValueError):
            FabricSampler(*args)
