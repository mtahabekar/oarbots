# Independent testing and evidence

Run tests in the isolated localhost ROS domain configured by the test fixtures. They use synthetic measurements, fake TF and temporary MCAP files. They never start hardware drivers or publish robot commands.

## Numerical modules

After building/sourcing, these tests run without a ROS graph:

```bash
python3 -m pytest -q src/fabric_state_estimator/test/test_numerics.py
python3 -m pytest -q src/fabric_reference_reconstruction/test/test_reference.py
```

To test directly from source without building ROS interfaces:

```bash
export PYTHONPATH="$PWD/src/fabric_perception:$PWD/src/fabric_state_estimator:$PWD/src/fabric_reference_reconstruction:$PYTHONPATH"
python3 -m pytest -q src/fabric_state_estimator/test/test_numerics.py src/fabric_reference_reconstruction/test/test_reference.py
```

| Behavior | Evidence |
|---|---|
| Persistent material identity | UV interpolation, fixed topology, model hash and exact mass sum |
| CPU physics contract | Internal-force conservation, gravity motion, prescribed pins and distributed-load cancellation |
| Ensemble mathematics | Local transform mean and q/v covariance agree with a closed-form linear Kalman update |
| Visibility | Overlapping/folded geometry test retains the frontmost triangle and excludes the hidden layer |
| Depth semantics | Tangential displacement does not change a pure normal factor |
| Tracking semantics | Track birth supplies no self-measurement; later pixels use a persistent binding; zero-confidence occlusion edges do not become boundaries |
| Causality and forecasts | Backward captures fail, branches do not mutate analysis, measurement stamps stay fixed, long gaps reinitialize |
| Recording integrity | Exact integer track stamps survive JSON round trips; malformed/reversed sequences fail |
| Recorded transforms | Interpolated capture-time transform differs from latest transform; extrapolation fails |
| Offline non-causality | Future raw depth improves the initially hidden surface and reduces normalized residuals |
| Validation | Independent truth matching, partial-truth coverage, observed/inferred error separation and position-covariance coverage |

## Newton backend

The Newton test is optional in the ordinary ROS environment and reports a skip if Newton is absent. Run it explicitly in the pinned environment; a skip is not validation of that backend:

```bash
export WARP_CACHE_PATH=/path/to/writable/warp-cache
python -m pytest -q src/fabric_state_estimator/test/test_newton.py
```

It checks independent worlds against a single-world result, prescribed attachment positions, distinct external loads, gravitational cancellation and agreement with the installed Style3D stretch/bend force kernels. This catches accidental cross-world collision, a mismatched reaction operator or clearing the residual force after it was written. Run on CPU first, then extend profiling/validation on the actual CUDA device. The included test explicitly uses CPU; passing it does not establish GPU throughput.

## ROS and MCAP adapters

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/fabric_state_estimator/test/test_ros_observer.py
python3 -m pytest -q src/fabric_reference_reconstruction/test/test_bag.py
```

The ROS test checks latched model delivery, compact observations and human grips through a world-frame mesh estimate, q/v covariance shape, source measurement stamps, forecasts and stopped publication beyond the prediction deadline. The MCAP tests check late recorded TF, raw glove inputs, actual storage serialization/deserialization and exclusion of online fabric estimates from reference inputs.

These new ROS tests use nodes in one process. The existing platform suite also exercises cross-process DDS and sensor adapters. A sandbox that denies localhost sockets can prevent those tests from communicating; run the isolated suite in an environment with localhost DDS enabled rather than treating missing messages as an estimator-algorithm failure.

If the existing cross-process RGB test receives CameraInfo but times out on image/marker delivery with the default transport mix, isolate the test environment using UDP-only Fast DDS:

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 bash scripts/check_platform.sh
```

Fast DDS documents this [builtin-transport environment setting](https://fast-dds.docs.eprosima.com/en/stable/fastdds/env_vars/env_vars.html#fastdds-builtin-transports). It affects the invoked test processes, not repository or robot configuration. The ArUco integration test passed with this setting after timing out with default transports in the development environment; this identifies a useful transport isolation step, not a complete diagnosis of that environment's default-transport issue.

## Full checks and package tests

```bash
python3 -m pytest -q src/fabric_state_estimator/test src/fabric_reference_reconstruction/test
colcon test --packages-select fabric_state_estimator fabric_reference_reconstruction
colcon test-result --verbose
bash scripts/check_platform.sh
```

The full platform check includes the earlier perception and human-tracking tests. CI includes both estimator packages. ROS/Python dependencies must be built and sourced; the platform tests additionally require the repository's normal GUI/vendor dependency environment.

## What remains to establish on hardware

Software verification does not establish observability or calibrated uncertainty. Before using estimates as reference labels, model-identification evidence or controller state, collect independent truth and evaluate:

- Visible/inferred/stale position p50/p95/max and fraction of the sheet with truth.
- Tangential/material correspondence error separately from normal surface error.
- Velocity and conditional forecast errors, with matched material identities and times.
- Capture-to-analysis/state age and end-to-end latency percentiles, including network and TF waits.
- Empirical covariance coverage and innovation statistics on held-out clips.
- Behavior during tag/depth/camera loss, initialization error, corner swaps and unmodeled contact.
- Force-frame/sign/compensation residuals in the intended slow-motion range.

Use the [hardware guide](hardware.md) for the setup procedures and [usage guide](usage.md#independent-truth-evaluation) for the evaluation format. Split data by whole trajectory and operating condition. Do not use this estimator or its simulator prior to manufacture the independent truth used to validate itself.
