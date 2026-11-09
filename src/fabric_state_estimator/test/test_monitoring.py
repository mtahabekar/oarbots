from dataclasses import replace
from fabric_state_estimator.monitoring import MonitorPolicy, MonitorSpec


def healthy(**changes):
    return dict(degraded=False, reinitialized=False, depth_factors=40,
                marginal_innovation_score=1., prior_depth_p95_m=.006, **changes)


def ready(policy):
    for k in range(1, 22):
        policy.update(k*200_000_000, healthy(), ['left', 'right'])
    return 4_200_000_000


def test_warmup_requires_elapsed_time_diverse_views_and_consecutive_good_evidence():
    policy = MonitorPolicy(MonitorSpec(enabled=True))
    for k in range(1, 18):
        policy.update(k*200_000_000, healthy(), ['left'])
    assert policy.mode(3_400_000_000) == 'warmup'
    policy.reset()
    for k in range(1, 11):
        policy.update(k*200_000_000, healthy(), ['left', 'right'])
    assert policy.mode(2_000_000_000) == 'warmup'
    assert policy.mode(ready(policy)) == 'monitor'


def test_bad_depth_large_innovation_reinitialization_and_dropout_restore_sampling():
    for key, value in [('prior_depth_p95_m', .04), ('prior_depth_p95_m', float('nan')),
                       ('marginal_innovation_score', 8.), ('depth_factors', 0),
                       ('reinitialized', True), ('degraded', True)]:
        policy = MonitorPolicy(MonitorSpec(enabled=True)); t = ready(policy)
        status = healthy(); status[key] = value
        policy.update(t+1_000_000_000, status, ['left', 'right'])
        assert policy.mode(t+1_000_000_000) == 'warmup'
    policy = MonitorPolicy(MonitorSpec(enabled=True)); t = ready(policy)
    assert policy.mode(t+3_000_000_000) == 'warmup'
    policy = MonitorPolicy(MonitorSpec(enabled=True)); t = ready(policy)
    assert policy.mode(t-1_000_000_000) == 'warmup'


def test_low_rate_remains_valid_without_restarting_warmup_on_each_check():
    policy = MonitorPolicy(MonitorSpec(enabled=True)); t = ready(policy)
    for k in range(1, 20):
        policy.update(t+k*1_000_000_000, healthy(), ['left', 'right'])
        assert policy.mode(t+k*1_000_000_000) == 'monitor'
    disabled = MonitorPolicy(replace(policy.spec, enabled=False)); t = ready(disabled)
    assert disabled.mode(t) == 'warmup'
