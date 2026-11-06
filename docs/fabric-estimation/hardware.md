# Preparing and using real hardware

This guide covers the current freely hanging, four-corner case: fabric already attached to both robots; both tagged human gloves holding their corners throughout. Run the estimator in shadow mode. Neither estimator package commands motion or connects itself to the controller.

## 1. Record the physical configuration

Create one deployment directory containing camera YAMLs, hand-fusion YAML, estimator YAML, all calibration files and model files. Use absolute paths in launch commands. Record the date, repository revision, uncommitted diff if any, OS/ROS/Python versions, Newton/Warp versions, camera serial numbers, frame names, sheet ID and glove/tag IDs. Save `python -m pip freeze` from the runtime environment.

Measure and record:

| Quantity | Procedure and use |
|---|---|
| Rest width/height | Measure the unstretched ply along the selected material u/v directions. Set `mesh.width_m`, `height_m`. Defaults are simulator examples. |
| Dry mass and area | Weigh the actual sheet; compute `areal_density_kg_m2 = mass_kg/(width_m*height_m)`. Exclude grippers/tags/gloves from sheet mass. |
| Weave orientation | Identify warp/weft and map them to u/v. Record which physical corner is `(0,0)`. Do not rotate the sheet or swap attachments without updating the mapping. |
| Attachment locations | Define the cloth contact point at Blue `(0,0)`, Silver `(1,0)`, human left `(0,1)`, human right `(1,1)`. This version models point corners, not finite contact patches. |
| Operating envelope | Record camera distances, sheet speed/acceleration, expected sag and permitted slack. No self-contact, table contact or obstacle contact is supported. |
| Material coefficients | Start with nominal Style3D values only for shadow tests. These are effective solver coefficients, not cured-composite elastic moduli. Joint parameter identification is not provided yet. |

Use a resting/slow initial configuration with all four corners measured and the surface close to the configured sag prior. The observer does not solve arbitrary folded-sheet initialization. Keep bases stationary for first tests, then validate moving-base TF separately.

## 2. Newton and the ROS Python environment

Use Python 3.12 with ROS Jazzy on the cell. The CPU spring fixture requires no Newton installation. The recommended backend is version-tested against Newton 1.0.0 and Warp 1.12.0. Newton's official [versioned installation guide](https://newton-physics.github.io/newton/1.0.0/guide/installation.html) describes the base pip package and virtual-environment workflow. This adapter does not need Isaac Sim or the examples extras.

Create a ROS-compatible virtual environment; keep NumPy below 2 to match the system OpenCV/cv_bridge ABI:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages /path/to/fabric-runtime
source /path/to/fabric-runtime/bin/activate
python -m pip install -r /path/to/oarbots/src/fabric_state_estimator/config/requirements-newton.txt
python -c 'import numpy, cv2, rclpy, newton, warp; print(numpy.__version__, cv2.__version__, newton.__version__)'
```

Build using **the same Python interpreter**. Activating a venv after building does not change an existing console script's shebang. Use the venv interpreter to invoke colcon:

```bash
python -m colcon build --packages-up-to fabric_state_estimator fabric_reference_reconstruction
source install/setup.bash
head -n 1 "$(ros2 pkg prefix fabric_state_estimator)/lib/fabric_state_estimator/observer"
```

The shebang should name the intended environment. If the workspace has old Python build artifacts, use a fresh build/install base or rebuild those packages in the selected environment. Do not mix an Isaac/ML environment with arbitrary NumPy/OpenCV versions into ROS via a broad `PYTHONPATH` override.

Before selecting `device: cuda:0`, verify that Warp can access that device in the runtime environment:

```bash
python - <<'PY'
import warp as wp
wp.init()
print(wp.get_devices())
PY
```

Then run the explicit Newton backend test in [testing.md](testing.md). `device: cpu` runs the same adapter without CUDA but is not a real-time performance claim. First launch compiles kernels; the live node warms the backend before accepting captures. Keep the Warp kernel cache writable (`WARP_CACHE_PATH` can point to a deployment cache directory).

GPU runtime, driver compatibility and throughput have not been tested in this development environment. Pin the validated versions, record device information, and measure the actual host update + device transfer + physics cost. Do not assume 32 members at 17×17 meet 15 Hz because a smaller software fixture does.

## 3. Cameras, clocks and registration

Follow the camera configuration in [fabric-perception.md](../fabric-perception.md) and gates H7/H9/H11/H15 in the [platform audit](../platform-audit.md). Required per-camera inputs are raw RGB, its CameraInfo and depth registered into the RGB optical frame at the same resolution. Raw depth in the depth-camera frame is not a valid replacement.

1. Identify cameras by serial and lock the intended frame prefixes. All hosts must agree on the common world frame and ROS clock domain.
2. Calibrate RGB intrinsics/distortion at working distance. Validate registered depth against known geometry; check edge alignment and that metre/mm encodings are correct. Supported encodings are `16UC1` mm and `32FC1` m.
3. Calibrate static-camera extrinsics and wrist hand–eye transforms using diverse poses, including loaded hardware. Publish them in TF; a CAD mounting transform is an initial guess, not validation.
4. Validate the complete chain from world → base localization/odometry → arm → wrist camera. The NUC base-localization transform may arrive after a wrist image. The fusion/observer buffers wait for bracketing sample-time TF, not latest TF.
5. Configure wired camera synchronization and fixed exposure where available, and independently synchronize host clocks. Measure RGB/depth offsets, camera-to-camera capture skew and inter-host clock error. Approximate message pairing is not hardware synchronization.
6. Record the actual carbon sheet at all planned ranges, angles and lighting conditions. Measure ToF valid fraction, bias/noise, glare, boundary visibility and track retention. Keep raw IR in the bag for later depth-training work. Missing depth is acceptable only if the remaining observations and uncertainty support the intended experiment.

At speed `s`, timestamp uncertainty `dt` permits roughly `s*dt` positional error. Allocate the timing/calibration share of the error budget before choosing `camera_sync_tolerance_sec` and noise floors. The default 15 ms grouping tolerance is not a statement that the physical cameras are aligned to 15 ms.

On the helper laptops, run camera perception locally. Send compact observations/glove measurements over DDS. Record high-bandwidth raw images locally or on a network that has been measured to support them; three uncompressed high-resolution streams can exceed the cell Wi-Fi budget. Verify cross-host topic discovery, QoS and throughput with the actual DDS configuration. The test suite's localhost domain 197 is only for isolated tests, not the cell deployment domain.

## 4. Calibrated robot attachment frames

The estimator expects two explicit TF origins at the actual cloth contact points, named in `runtime.robot_grip_frames`. Defaults are `oarbot_blue/fabric_grip` and `oarbot_silver/fabric_grip`. These frames do not exist automatically just because an end-effector frame exists. Choose a world origin near the workcell to preserve Newton float32 precision; do not use geographic-scale coordinates directly as particle positions.

Determine the rigid transform from each measured robot end-effector/tool frame to its fabric attachment point. Use a fixed target or metrology fixture over multiple arm poses, and verify against an independent camera/target measurement. Include mount compliance and tool/camera load in the residual assessment. Set `runtime.robot_position_std_m` from held-out errors rather than leaving the 5 mm example unquestioned.

For a fixed attachment transform, publish it using your cell launch/calibration file. The equivalent CLI form is below; set all shell variables from the measured transform before use:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x "$BLUE_GRIP_X_M" --y "$BLUE_GRIP_Y_M" --z "$BLUE_GRIP_Z_M" \
  --qx "$BLUE_GRIP_QX" --qy "$BLUE_GRIP_QY" --qz "$BLUE_GRIP_QZ" --qw "$BLUE_GRIP_QW" \
  --frame-id oarbot_blue/j2n6s300_end_effector --child-frame-id oarbot_blue/fabric_grip
```

Repeat for Silver with its measured transform. Confirm the actual end-effector frame names from TF on your deployment. Publish each child from one authority only. This does not command the robot. Check each world-to-grip transform while moving slowly and verify that its origin follows the cloth corner rather than the tool flange.

## 5. Tagged gloves and pinch-point calibration

Use distinct persistent tags for human left/right, with a white quiet border and measured outer-black-square size. Example IDs are 20/21 in `DICT_4X4_50`, separate from robot localization IDs. Both gloves may share one distinctive colour. Use non-glove-coloured light sleeves and a contrasting background when using HSV sheet segmentation. A flexible or bent tag violates the planar pose model; prefer a rigid cuff/tag fixture.

For each glove, measure the vector from tag centre to the actual fabric pinch point **in tag coordinates**. OpenCV tag axes are x right, y up on the printed face, z out of that face. Enter left xyz then right xyz in `tag_to_grip_offsets_m`. A practical calibration is to hold the pinch point against a fixed metrology target while rotating the rigid tag fixture through several poses, solve for the constant offset, and validate on withheld poses. The current tool consumes the calibrated offset; it does not automate this calibration fit.

Measure tag reprojection residual, grip-position error against independent truth, dropout rate, covariance coverage and cross-view agreement. Set size, offsets, colour bounds, support/area gates and noise floors from those measurements. Zero offsets and the green HSV bounds in the example camera YAML are not calibrated values.

Set the hand-fusion world frame to the estimator's world frame and `human_corner_uv: [0.0,1.0,1.0,1.0]`. Both hand states must say HOLDING and have these material identities. The estimator propagates only a fresh causal hand state, with its velocity covariance, for at most `hand_max_age_sec`. It will not use a future hand measurement to initialize an earlier fabric capture. If a tag drops out, it waits/drops the frame rather than inventing a new hand measurement.

Holding is assumed for this task. Releasing a corner, swapping corners or slipping invalidates the prescribed-boundary model; do not interpret a plausible-looking mesh as a release/slip diagnosis.

## 6. Segmentation and exclusion masks

Prepare one perception YAML per camera. Tune the ROI, glove hue, dark-fabric thresholds or trained segmenter, component area and depth gates on recorded frames. Inspect every operating view, including wrists tilted toward the human.

For a wrist's own fixed gripper, paint `static_exclusion_mask` at raw RGB resolution using a representative recorded image. For moving robot pixels, supply an exact-capture `fabric/robot_mask` and set `require_robot_mask=true`, or use a validated segmenter's robot class. Ensure exclusion edges are zero-confidence boundaries. Do not include a predicted fabric silhouette as an independent measured mask.

Enable `publish_debug_images` during commissioning. Check `fabric/labels`, `fabric/depth_valid`, contours and valid-depth fraction. Then disable full-resolution debug image transport for normal operation as needed. The baseline cannot distinguish arbitrary dark sleeves, table surfaces or robot parts from dark fabric merely by colour.

## 7. Optional wrist-force factors

Leave `runtime.use_forces: false` until platform gates H5/H6 pass. Confirm device identity, sensor axes, sample stamps, tool mass/centre of mass, bias, gravity compensation, drift, and force sign. Use the calibrated topic named in the config, not raw uncorrected wrench or admittance commands/deadbanded forces.

The factor expects **force on the sheet**. The example negates a cloth-on-tool measurement using `force_on_sheet_sign: -1.0`. Verify with known loads/directions in the world frame. The source frame must be the documented measurement frame, and its TF must exist at the sensor sample time. Set `force_stddev_n` from held-out residuals and `force_max_age_sec` from measured timing, not by making stale forces look fresh.

This first factor uses force only. It does not use torque, finite-patch moments, grip compliance, sensor inertial compensation or dynamic grip acceleration. Restrict initial force-assisted experiments to quasi-static/slow operation, and compare predicted reactions with measured values before trusting them. Forces are residuals, never reapplied to kinematically prescribed corners.

## 8. Bring-up order and observations to inspect

1. Start robot sensing/TF, base localization, camera drivers and calibrated static transforms using the existing platform launch procedures.
2. Confirm clocks, camera serials, registered images, CameraInfo and dynamic world-to-camera/world-to-grip TF.
3. Start per-camera `fabric_perception camera.launch.py` on each camera host with its own YAML.
4. Start one `fabric_perception fusion.launch.py` on the estimator host. Verify both hand positions, source ages, identities and holding assumption.
5. Start one `fabric_state_estimator observer.launch.py` with the deployment config. Expect a geometric prior initially, followed by observation updates. It should wait if an attachment or transform is absent.
6. Inspect `/fabric/model`, `/fabric/estimated_state`, `/fabric/predicted_states` and `/fabric/estimator_status`. Plot sample residuals, q/v, covariance ellipsoids, direct-observation labels and ages. A model message is latched for late subscribers.
7. Record a stationary hold, then slow small motions, camera occlusions and deliberate sensor dropouts. Extend to moving bases only after validating the world-frame chain. Keep a separate independent truth stream for acceptance measurements.

The observer reports capture-to-analysis age. More than 100 ms triggers degraded diagnostics against the proposed research latency target; it is not a measured performance guarantee. `prediction_limit_sec` bounds live extrapolation. The forecasting timer does not advance the last measurement stamp. Inspect age in addition to topic rate: a high publish rate can still represent old information.

## 9. Recording, offline reconstruction and acceptance

Record the raw RGB/registered depth/IR, camera calibration, TF/TF-static, per-view glove observations, per-camera factors and calibrated wrenches, plus online output/status for comparison. The recorder script includes these and uses transient-local QoS for `/fabric/model` and `/tf_static`.

Use short calibration clips. Export with `import_bag`, inspect all dropped-capture reasons, run deterministic online replay, then run Tier A independently on the measured factors. Check convergence and normalized residuals before using a reference candidate. Neither low fitting residual nor agreement between two estimators validates the physical state by itself.

For physical acceptance, use temporary material-indexed markers/mocap or independent calibrated scans on a sacrificial ply, with additional calibrated cameras where necessary. Validate visible and inferred regions separately, position/velocity errors, forecast errors at each horizon, capture-to-state latency, direct-observation age, empirical covariance coverage and reacquisition after dropouts. Repeat across the declared lighting, angles, sag, speed and base-motion envelope. Supply matched material indices and timestamps to `evaluate`; report the fraction of vertices/time samples for which independent truth actually exists.

The research proposes online whole-sheet p95 ≤50 mm and offline visible p95 around 10–15 mm, subject to parameter sensitivity. Those are **proposed targets**, not results of this implementation. Tier A currently has no state/parameter posterior, and its fitted trajectory is not automatically a calibration/identification reference. Keep controller/twin consumers disconnected until the required hardware evidence and uncertainty margins have been established for the operating envelope.

## Troubleshooting

| Symptom | Inspect or correct |
|---|---|
| No model/node startup | Runtime Python shebang, pinned Newton imports, writable Warp cache, selected device, configuration parse errors |
| Model but no state | Both holding hands at correct UV, common world frame, robot grip frames, capture-time camera TF, nonzero source stamps |
| `Missing causal, fresh human grip measurements` | Hand stream timestamps and latency; future samples are intentionally excluded; clock sync; `hand_max_age_sec` |
| Late/expired captures | Clock offsets, processing/network latency, TF publication lag, inactive-camera timeouts, wait/age settings |
| Few/no visual factors | Segmentation/ROI, carbon depth dropout, camera registration, initial sag, association gates, incorrect optical TF |
| Low ensemble spread | Initial/process spread, overconfident observations, insufficient ensemble size; review coverage rather than only inflating displayed covariance |
| Large innovations or edge-strain violation | Wrong sheet dimensions/identity, swapped corners, slipping/release, calibration error, unmodeled contact or wrong material |
| Import cannot initialize | Recorded raw `fabric/gloves`, both human corners within synchronized groups, robot grip frames and TF history; fused hand states are intentionally not an input |
| Reference does not converge | Shorter/coarser clip, better seed/calibration, valid factor coverage, model-noise scales, sufficient evaluation budget; inspect sidecar |
| Good residual, bad truth error | Incorrect correspondence, calibration bias, model-prior bias, missing tangential information or non-independent truth; do not accept based on residual alone |
