# Build, replay and reconstruction

## CPU software installation

Run from a ROS 2 Jazzy workspace with this repository under its `src` directory. Resolve declared ROS dependencies using the repository's normal rosdep workflow. New numerical dependencies are `python3-numpy`, `python3-scipy`, `python3-opencv` and `python3-yaml`; bag import also needs `rosbag2_py` and the storage plugin used by the recording.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to fabric_state_estimator fabric_reference_reconstruction
source install/setup.bash
```

The CPU fixture and numerical tests do not need Newton, a GPU, Isaac Sim, cameras or robots. For the production Newton environment, use the separate [hardware installation procedure](hardware.md#newton-and-the-ros-python-environment).

## Complete synthetic walkthrough

These commands run installed tools. Choose a new output directory; the numerical result files are overwritten if the same output paths are reused.

```bash
mkdir -p /tmp/fabric-mesh-demo
ros2 run fabric_reference_reconstruction synthetic /tmp/fabric-mesh-demo --rows 3 --frames 5
ros2 run fabric_state_estimator replay /tmp/fabric-mesh-demo/sequence.json /tmp/fabric-mesh-demo/online.npz --ensemble-size 12
ros2 run fabric_reference_reconstruction reconstruct /tmp/fabric-mesh-demo/sequence.json /tmp/fabric-mesh-demo/reference.npz
ros2 run fabric_state_estimator evaluate /tmp/fabric-mesh-demo/online.npz /tmp/fabric-mesh-demo/truth.npz /tmp/fabric-mesh-demo/online-metrics.json
ros2 run fabric_state_estimator evaluate /tmp/fabric-mesh-demo/reference.npz /tmp/fabric-mesh-demo/truth.npz /tmp/fabric-mesh-demo/reference-metrics.json
```

`sequence.json` contains only measurements, nominal model configuration and provenance. `truth.npz` is separate and is never read by replay or reconstruction. These are deterministic software fixtures generated with the small spring plant, not a substitute for carbon-fabric measurements or independent physical validation. Do not interpret their errors as hardware accuracy.

Inspect artifacts without ROS:

```bash
python3 - <<'PY'
import json
import numpy as np
with np.load('/tmp/fabric-mesh-demo/online.npz', allow_pickle=False) as data:
    print({key: data[key].shape for key in data.files})
print(json.load(open('/tmp/fabric-mesh-demo/reference.npz.json')))
PY
```

Replay output includes `times_ns[T]`, `positions[T,V,3]`, `velocities[T,V,3]`, `covariance[T,V,6,6]`, `visibility[T,V]`, `observation_age_sec[T,V]` and `residual_forces[T,3]`. Reference output includes `times_ns`, `positions`, `velocities` and `residual_forces`; it does not provide posterior covariance. Both have JSON sidecars. A reference process exit code 2 means it did not converge; inspect the written report before accepting the trajectory.

The pure tools also support `python -m fabric_state_estimator.replay`, `python -m fabric_state_estimator.metrics`, and `python -m fabric_reference_reconstruction.{synthetic,cli}`. If using source directly, add `src/fabric_perception`, `src/fabric_state_estimator` and `src/fabric_reference_reconstruction` from this repository to `PYTHONPATH`. Their numerical modules need no initialized ROS context. `bag` additionally requires a sourced ROS environment.

## Configuration

Copy [estimator.yaml](../../src/fabric_state_estimator/config/estimator.yaml) to a deployment-specific file. Its 0.84 m sheet, 0.3 kg/m² density, material coefficients and noise values are examples. The default selects Newton on `cuda:0`; it will not silently fall back to the spring plant. For a software-only live node, copy [cpu_demo.yaml](../../src/fabric_state_estimator/config/cpu_demo.yaml).

This is a plain structured YAML file with `mesh`, `physics`, `observer`, `observations`, `runtime` and `reference` sections, **not** a ROS `ros__parameters` file. The node accepts one ROS parameter, `config_file`. Unknown sections and unknown dataclass fields are rejected. Restart the node after changing its configuration; dynamic parameter reconfiguration is not implemented.

| Section/key | Units and purpose |
|---|---|
| `mesh.width_m`, `height_m` | Measured rest dimensions, metres; u/warp and v/weft convention must match the ply |
| `mesh.rows`, `columns` | Number of persistent vertices per direction, not cells; 17×17 is the proposed online starting point |
| `mesh.areal_density_kg_m2` | Dry sheet mass divided by rest area |
| `physics.backend`, `device` | `spring` or `newton_style3d`; Newton `cpu` or e.g. `cuda:0` |
| `physics.max_step_sec`, `iterations` | Physics substep upper bound and Style3D nonlinear iterations |
| `physics.gravity` | World gravity vector in m/s², normally `[0,0,-9.81]` |
| `physics.newton_stretch`, `newton_bend` | Style3D warp/weft/shear and directional bend coefficients; solver-specific, fixed nominal values until identified |
| `physics.spring_stiffness_n_m`, `damping_per_sec` | CPU test-model spring N/m and velocity damping s⁻¹; do not use to calibrate Newton |
| `observer.ensemble_size` | At least 4; start 32, profile before increasing to 64 |
| `observer.localization_m` | Material-geodesic taper radius; support reaches twice this value |
| `observer.inflation` | Ensemble covariance inflation in the transform update |
| `observer.initial_sag_m`, `initial_position_std_m`, `initial_velocity_std_mps` | Geometric initialization and ensemble spread assumptions |
| `observer.residual_force_std_n` | Initial total-force coefficient standard deviation |
| `observer.residual_force_walk_n_sqrt_sec` | Random-walk coefficient scale, N/√s |
| `observer.velocity_noise_mps_sqrt_sec` | Velocity process-noise scale, (m/s)/√s |
| `observer.stale_after_sec` | Vertex observation-age threshold for STALE labels |
| `observer.max_gap_sec` | Capture gap triggering geometric reinitialization |
| `observer.forecast_limit_sec` | Maximum branch-rollout duration from last analysis; must cover prediction age plus forecast horizons |
| `observer.max_edge_strain` | Allowed overextension fraction during projection; not a calibrated material locking limit |
| `observer.covariance_floor_m`, `seed` | Published position uncertainty floor and reproducible random seed |
| `observations.max_depth`, `max_boundary`, `max_tracks` | Bounded number of retained per-view samples; tracks/UV emit two scalar pixel factors each |
| `observations.association_gate_m` | Metric gate between visible mean/reference surface and measured point |
| `observations.visibility_tolerance_m` | Separate front-layer visibility tolerance for projected boundary/track/UV stencils; default 0.02 m, not the wider association gate |
| `observations.pixel_stddev`, `depth_floor_m` | Image residual px standard deviation and extra depth/calibration floor in metres |
| `observations.max_surface_speed_mps` | Bound used to inflate variance for grouped-view timing skew; declare the actual operating range |
| `observations.use_uv` | False until independent UV validation; optional priors only |
| `observations.max_track_bindings` | Bounded persistent camera-local material-binding history, default 2000 |
| `runtime.world_frame`, `robot_grip_frames` | Common frame and Blue/Silver calibrated attachment origins |
| `runtime.observation_topics`, `hand_topic` | Per-camera factor topics and fused human hand states |
| `runtime.robot_position_std_m` | Robot attachment calibration uncertainty |
| `runtime.hand_max_age_sec` | Maximum causal hand-state age; covariance propagates during this interval |
| `runtime.max_wait_sec`, `max_capture_age_sec`, `reorder_wait_sec` | Capture-relative dependency wait, hard observation age, and short arrival reordering delay |
| `runtime.camera_sync_tolerance_sec` | Offline multi-camera grouping tolerance; original per-view stamps/transforms remain stored |
| `runtime.prediction_limit_sec`, `publish_rate_hz`, `forecast_horizons_sec` | Live prediction age bound, timer rate and future horizons from publication origin |
| `runtime.use_forces`, `force_topics` | Explicit opt-in to the two tool-compensated robot forces |
| `runtime.force_on_sheet_sign` | `−1` for cloth-on-tool sensor convention; validate physically, only ±1 accepted |
| `runtime.force_stddev_n`, `force_max_age_sec` | Force residual noise and maximum causal sample age |
| `reference.position_model_std_m`, `velocity_model_std_mps` | Weak-dynamics residual scales **per recorded interval**; retune when sampling rate changes |
| `reference.initial_position_std_m`, `initial_velocity_std_mps` | Initial-state priors; avoid fitting an online fabric trajectory as truth |
| `reference.residual_force_std_n`, `residual_force_walk_n` | Offline force magnitude and adjacent-frame increment priors |
| `reference.huber_threshold` | Robust threshold in normalized measurement-residual units |
| `reference.max_nfev`, `association_rounds`, `initial_sag_m` | Inner evaluation budget, reassociation rounds and geometric seed |
| `reference.max_variables`, `max_jacobian_entries` | V1 memory/problem-size guards, default 12,000 variables and 20,000,000 estimated sparse entries; reduce mesh/clip before increasing these |

All noise and spread defaults require held-out validation. Increasing queue age limits can prevent drops while increasing latency; it does not improve synchronization or make the predictor's information newer.

## Live shadow observer

After completing hardware preparation and launching per-camera perception plus hand fusion:

```bash
ros2 launch fabric_state_estimator observer.launch.py config_file:=/absolute/path/to/cell-estimator.yaml
ros2 topic echo /fabric/estimator_status
ros2 topic hz /fabric/estimated_state
```

The observer waits for calibrated camera and robot-grip TF plus two causal holding-hand states. It does not initialize from zero positions if those inputs are absent. Its output is visualization/replay/shadow data; no bridge to CBF, velocity commands or the fine simulator is provided.

For a live mesh display, run the optional RViz adapter and start RViz (if installed):

```bash
ros2 run fabric_state_estimator visualize
ros2 run rviz2 rviz2
```

Set RViz's Fixed Frame to the configured `world_frame`. Add a MarkerArray display subscribing to `/fabric/mesh_markers`. Blue edges touch inferred vertices, green touches directly observed vertices, orange touches stale vertices; a degraded state is red. These colours indicate observation/health labels, not demonstrated accuracy. Markers expire after 0.5 seconds without a new state. During live bag replay pass `--ros-args -p use_sim_time:=true` to the display as well.

## Import and fit a real recording

Record after all nodes have started using `scripts/record_fabric_baseline.sh`. Archive every camera YAML, hand-fusion YAML, estimator YAML, model weights, calibration files and environment versions beside the bag.

```bash
ros2 run fabric_reference_reconstruction import_bag /path/to/bag /path/to/cell-estimator.yaml /path/to/sequence.json --storage-id mcap --max-frames 60 --stride 2
ros2 run fabric_state_estimator replay /path/to/sequence.json /path/to/online-replay.npz --config /path/to/cell-estimator.yaml
ros2 run fabric_reference_reconstruction reconstruct /path/to/sequence.json /path/to/reference.npz --config /path/to/cell-estimator.yaml
```

Start Tier A with a 3×3 or 5×5 mesh configured **before import** and a short clip. A long 17×17 solve can exceed the v1 finite-difference problem budget and is rejected before allocation. To change mesh resolution, re-import the measured factors with the new mesh config; replay/reconstruction intentionally reject a mesh-ID mismatch. Larger meshes need a deliberately provisioned solve or a future more efficient reference backend. The importer's `--stride` selects capture groups; original source timestamps remain unchanged. Offline model-noise scales are per retained interval and need adjustment after changing stride. Long gaps over two seconds require separate clips. `--max-frames` bounds exported optimization length; use short bags because importer TF/compact-observation history is held in memory.

Importer inputs are recorded `/tf`, `/tf_static`, configured per-camera observation topics, corresponding `fabric/gloves` arrays, and optionally the configured calibrated force topics. Both human corners must be available for initialization. The importer excludes online fabric states and fused hand states. Dropped captures and reasons are saved in sequence provenance. Poor coverage, unknown grip frames or absent raw glove topics require correcting the recording before fitting.

For live bag replay, use `use_sim_time:=true` on perception, hand fusion and observer, and `ros2 bag play --clock`. Remap the original recorded fabric-state/model/forecast/status topics away from the newly computed ones, or replay only raw/camera-factor/hand/TF inputs. Restart nodes after backward seeks or loops. The offline `replay` command avoids DDS clocks and topic collisions entirely.

## Independent truth evaluation

Truth NPZ must contain exactly matching `times_ns[T]` and material-indexed `positions[T,V,3]`. Use NaNs for vertices without independent truth; do not fill hidden vertices using this estimator. An external preprocessing/calibration step must map truth markers or scans to the persistent material indices and capture clock. The evaluator rejects mismatched timestamps and shapes rather than guessing correspondence.

```bash
ros2 run fabric_state_estimator evaluate /path/to/online-replay.npz /path/to/independent-truth.npz /path/to/metrics.json --max-p95-m 0.05
```

The optional threshold makes the command exit 2 if all-evaluated-point p95 exceeds the limit. Reports include truth coverage, overall and observed/inferred/stale p50/p95/max, and empirical position ellipsoid coverage when covariance exists. A sparse visible-only truth set cannot validate whole-sheet or hidden-surface performance; the reported coverage fraction makes that limitation explicit.
