# Online bank parallel replay check, 2026-11-21

Offline implementation and validation; no robot motion or cell-calibration changes.

The bank now evaluates warm-up candidates in independent spawned processes, with one Style3D model per worker
and CUDA MPS for GPU sharing. The search objective, ordering, convergence probes and material-release conditions
are unchanged. Worker count is capped from available host memory (6 GB each, 4 GB reserved); missing MPS falls
back to sequential execution. Default: one worker; opt in with `BANK_WORKERS=2` for the hardware wrapper, or
`search_workers:=2` for a direct compute launch. Slow adaptation after the warm-up remains sequential.

## Recorded comparison

- Source recording: `../2026-11-21_fabric_run/warmup_10_0.1_113825`.
- Configuration: that recording's `cell/twin.yaml`, not the current cell configuration.
- GPU: NVIDIA GeForce RTX 5070 Laptop GPU, CUDA MPS enabled for both paths.
- All 24 usable holds, identical cases and sampled raw depth for both paths.
- Start: bend x10, shear x0.1; span 8, maximum 4 rounds; **6 candidate evaluations per path**.
- Time limit disabled for comparison, to ensure identical candidates. Timings include worker startup and cleanup.

| Path | Effective workers | Seconds | Best bend | Best shear | RMS, mm | Stop |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Sequential | 1 | 123.759 | 1.25 | 0.0625 | 20.710579 | evaluation_limit |
| Parallel | 2 | 90.975 | 1.25 | 0.0625 | 20.710809 | evaluation_limit |

**Passed:** same candidates, winner and stopping decision. Largest candidate-cost difference: 0.000230 mm;
sensitivity difference: 0.000546 mm (both below the 0.1 mm comparison tolerance). Measured speedup: **1.36x**.
This is a bounded parity check, not a demonstration of full-search convergence or live-cell timing under load.
It does not fit glove offsets or implement the proposed joint corner-weighted objective.

Machine-readable results: `comparison.json`; console output: `replay.log`. The temporary MPS daemon used for
this check was stopped afterwards. Reproduce with the hardware wrapper's `bank-replay` command documented in
`scripts/hw/README.md`; raise `--max-evaluations` to 80 for a full search comparison.

## Software validation

88 tests passed across `test_bank_search.py`, `test_bank_parallel.py`, `test_warmup.py`,
`test_warmup_control.py`, and `test_warmup_pose.py`, with ROS dependencies sourced. These cover search parity,
ties, invalid costs, boundary optima, evaluation/time budgets, memory/MPS fallback, worker errors/death,
cleanup on success/failure/interruption, and release gating. The existing disarm-before-speech test fixture was
updated to simulate the current toggle-deadman press and completion prompt; production deadman code was unchanged.
`fabric_twin` and `fabric_cell` built successfully with the fabric ROS venv.
