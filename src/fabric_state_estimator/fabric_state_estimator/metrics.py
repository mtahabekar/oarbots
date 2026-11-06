"""Independent material-indexed truth comparison; never used as estimator input."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.stats import chi2


def evaluate(estimate, truth):
    times, expected_times = np.asarray(estimate['times_ns']), np.asarray(truth['times_ns'])
    q, expected = np.asarray(estimate['positions']), np.asarray(truth['positions'])
    if q.shape != expected.shape or q.ndim != 3 or q.shape[2] != 3 or not np.array_equal(times, expected_times):
        raise ValueError('Truth must have the same material indices, shape and capture timestamps; no implicit resampling')
    valid = np.isfinite(q).all(axis=2) & np.isfinite(expected).all(axis=2)
    if not valid.any():
        raise ValueError('No finite independent truth points')
    errors = np.linalg.norm(q-expected, axis=2)
    def summary(mask):
        values = errors[mask & valid]
        return None if not len(values) else dict(count=len(values), p50_m=float(np.percentile(values, 50)),
            p95_m=float(np.percentile(values, 95)), max_m=float(values.max()))
    result = {'all': summary(valid), 'truth_coverage_fraction': float(valid.mean())}
    if 'visibility' in estimate:
        labels = np.asarray(estimate['visibility'])
        if labels.shape != valid.shape:
            raise ValueError('Visibility shape does not match positions')
        for name, label in [('inferred', 0), ('observed', 1), ('stale', 2)]:
            result[name] = summary(labels == label)
    if 'covariance' in estimate:
        covariance = np.asarray(estimate['covariance'])
        if covariance.shape != q.shape[:2]+(6, 6):
            raise ValueError('Expected full per-vertex 6x6 covariance blocks')
        c = covariance[valid, :3, :3]; delta = (q-expected)[valid]
        if not np.isfinite(c).all() or np.any(np.linalg.eigvalsh(c) <= 0):
            raise ValueError('Position covariance must be finite and positive definite')
        distance = np.einsum('ni,ni->n', delta, np.linalg.solve(c, delta[..., None])[..., 0])
        result['position_95_percent_coverage'] = float(np.mean(distance <= chi2.ppf(.95, 3)))
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimate'); parser.add_argument('truth'); parser.add_argument('output')
    parser.add_argument('--max-p95-m', type=float)
    options = parser.parse_args(args)
    with np.load(options.estimate, allow_pickle=False) as estimate, np.load(options.truth, allow_pickle=False) as truth:
        report = evaluate(estimate, truth)
    Path(options.output).write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report))
    if options.max_p95_m is not None and report['all']['p95_m'] > options.max_p95_m:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
