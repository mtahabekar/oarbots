# ROS2 Jazzy platform audit and acceptance report

Audit date: 2026-11-04. Local baseline: `a6eb6fa` (the working tree initially contained only the earlier, untracked fabric research document). No robot was moved, no camera or physical serial device was opened, no installed service was changed, and no changes were pushed to GitHub. Builds and downloaded dependencies were isolated under `/tmp/oarbots-audit`.

**This is a software-hardening result, not a certification that the platform is bug-free or ready for unsupervised human–robot handling.** The software checks below pass. Deployment still requires measured camera extrinsics, device identities, force calibration, firmware/command-axis verification, and the hardware acceptance tests below. Fabric reconstruction, multi-camera human fusion, and the simulator observer remain the next implementation phase.

## Comparison with ROS1

Reference: [Assistive-Robotics at eefd0498486e6b0c6376941c4b115892d5b9c689](https://github.com/rpiRobotics/Assistive-Robotics/tree/eefd0498486e6b0c6376941c4b115892d5b9c689). I examined its complete repository/package inventory and compared the relevant launch, command-lock, human-tracking, calibration, localization and collaborative-controller sources against the Jazzy code. The current repository is a selective redesign, not a feature-complete translation.

| ROS1 capability | Jazzy finding and disposition |
|---|---|
| Kinova/Dingo drivers and robot description | Kinova is an external ROS2 fork; Dingo uses the separately provisioned Clearpath ROS2 system and domain bridge. The 14 original local ROS2 packages were inspected; a fifteenth, `oarbot_runtime`, now supplies guard and hand-adapter nodes. |
| `tf_camera_body_all_joints_broadcaster` and human localization EKFs | Missing from this port. The old code indexes marker arrays and uses a distance-based covariance heuristic; it does not establish true tracking confidence. Added a narrower timestamped hand-measurement adapter with explicit person selection. Multi-camera identity association, velocity estimation and EKF/observer fusion are **not implemented** by that adapter. |
| `twist_mux` stop/deadman locks | Missing equivalent enforcement in the direct ROS2 paths. Added a guard next to each robot driver, leases, finite-value checks, speed limits and explicit arm-mode selection. |
| Collaborative admittance | The ROS1 controller explicitly assumes two robots rigidly grasp a rigid object. It is not an appropriate fabric dynamics model. The existing ROS2 independent-arm controller was repaired into bounded damped admittance; it is not a port of the cooperative controller. |
| Redundancy resolution, base velocity control and 2D collision avoidance | Not present as equivalent local ROS2 components. No whole-body controller or collision avoidance has been invented in this audit. The current guarded path is slow supervised velocity operation. |
| UWB and robot-localization fusion | Not ported here. ArUco remains the current external base-localization source; marker loss must not be interpreted as a fresh pose. |
| Overhead ArUco/world TF | Replaced assumed height/yaw and unsafe IMU initialization with explicit calibrated static camera extrinsics; preserved marker capture timestamps and optical frames. |
| F/T processing | Kept this installation's existing calibration values, not older ROS1 payload numbers. Fixed packet handling, frame semantics, freshness and the Blue averaging error. |
| Recording and deployment | Added a baseline recording script, exact dependency revisions, reviewable patches, meaningful regression checks and explicit maintenance updates. |

Legacy evidence: [robot launch and mux](https://github.com/rpiRobotics/Assistive-Robotics/blob/eefd0498486e6b0c6376941c4b115892d5b9c689/src/assistive_launch/launch/oarbot_blue.launch), [human broadcaster](https://github.com/rpiRobotics/Assistive-Robotics/blob/eefd0498486e6b0c6376941c4b115892d5b9c689/src/tf_broadcasters/src/tf_camera_body_all_joints_broadcaster.cpp), [master human localization](https://github.com/rpiRobotics/Assistive-Robotics/blob/eefd0498486e6b0c6376941c4b115892d5b9c689/src/assistive_launch/launch/master_computer.launch), [rigid-object admittance](https://github.com/rpiRobotics/Assistive-Robotics/blob/eefd0498486e6b0c6376941c4b115892d5b9c689/src/assistive_controllers/src/admittance_controller_collaborative.py).

## Changes made

### Force/torque acquisition and calibration

- Replaced packed-union/unaligned serial decoding with explicit little-endian parsing. Partial frames survive between polls; CRC failures and garbage resynchronize without discarding arbitrary whole packets. CRC counters initialize deterministically. Polling is nonblocking and bounded.
- Made the serial device a parameter, defaulting to `/dev/sensor_bota_rokubi_ft_0`, with descriptor ownership and cleanup. Configuration failures fail startup with the actual error.
- Removed repeated publication of the previous raw sample as though it were new. Reject sensor error status and non-finite readings. Coalesce a drained batch to its newest valid sample; do not label an undrained backlog as current.
- Added raw/calibrated frame IDs and URDF measurement frames. `j2n6s300_ft_measurement` has the historical connector-axis orientation but the modeled sensor wrench origin. This preserves the calibration's force/torque rotation convention without pretending the moment is about the arm connector. Verify the physical origin in acceptance test H5.
- Transform gravity at the sample timestamp. Wait in a bounded queue for matching joint TF; expire samples after 100 ms by default. Never substitute a zero/uninitialized gravity vector or a latest transform after TF failure.
- Publish diagnostics for receipt age, CRC errors, invalid status, sensor ticks, coalesced frames and TF drops. F/T ROS timestamps are explicitly **host receipt timestamps**, not calibrated sensor acquisition time. Device ticks are diagnostic; device-to-host clock mapping remains a later precision improvement.
- Keep the level-base gravity assumption explicit through `gravity_frame`; use a calibrated gravity-aligned world frame when the base can tilt. Compensation remains quasi-static and does not remove payload inertial loads during fast handling.
- Move Blue/Silver payload mass, CoM and bias values from launch source to `oarbot_launch/config/oarbot_*_ft.yaml` without changing their numbers.
- Correct Blue's averaging: apply its axis rotation once to the completed mean, not repeatedly to the running sum. Reject empty data, non-finite data, insufficient excitation/rank and nonpositive estimated mass. Add bounded sampling/action waits, action acceptance/result checks, failure stop requests, proper shutdown, and optional exclusive-create parameter output.
- Calibration motion now requires explicit `allow_motion:=true`. It must run in supervised setup mode with reviewed poses; the supplied Silver CSV is not automatically suitable for Blue.

### Commands and operator interface

- New `oarbot_runtime/command_guard` runs on each helper. It accepts `commands/teleop/arm`, `commands/admittance/arm` and `commands/teleop/base`, and alone publishes the normal velocity path to the driver/base.
- Defaults: 150 ms input command timeout, 250 ms deadman/E-stop/mode heartbeat timeout, 100 Hz output. Limits are vector norms: arm 0.1 m/s and 0.3 rad/s, base 0.15 m/s and 0.3 rad/s. These are commissioning settings, not certified safe speeds or hard real-time guarantees.
- Startup, E-stop and heartbeat loss require a fresh deadman release/press before re-arming. Mode changes clear old commands. Invalid commands stop the guard. Base commands require a fresh timestamp and the robot's `base_link` frame; only planar components are applied. Multiple publishers on control inputs are diagnosed and stop output when detected.
- Standalone teleop nodes now use the guarded input topics, a required launch namespace, finite values and input timeouts. They no longer replay a last nonzero message indefinitely or command a hardcoded robot. Do not run a standalone forwarder and the GUI as simultaneous producers on the same input.
- Arduino input loss/garbage produces released deadman + asserted software E-stop. It no longer flushes unread events or retains the last active state forever. Required parameters and timeout are validated.
- Admittance uses measured sample intervals and a damped mass/inertia model with bounded output. It stops on stale wrench, stale enable lease, TF failure or disable. Linear force uses the actual arm-base transform. The Blue slash/mapping bug is fixed. Angular admittance is disabled until `angular_mapping_validated` is explicitly set after physical tests.
- Admittance moved from the tablet to the helper to keep force feedback near the sensor/driver.
- GUI publishers persist and renew mode/enable heartbeats. ROS callbacks and Qt widget changes now run on one thread to avoid shared-dictionary races. SpaceMouse/Arduino freshness is checked. Node indicators explicitly describe discovery, not calibration or sensor health.
- Homing, gripper position actions and the unimplemented base-home/body-following buttons are disabled in the guarded GUI. **This is a deliberate behavior change.** Position-action setup is separate: helper `setup_mode:=true` disables guard/admittance and enables the existing action adapters/MoveIt execution. That mode is for supervised commissioning, not human-contact autonomous operation. The driver still exposes raw APIs; the guard is not an access-control boundary or hardware E-stop.
- The tablet launch honors a configurable `gui_python` interpreter, so activating a PySide6 virtual environment actually works even when the installed console script was built with system Python.

### Timing, transforms, localization and camera configuration

- Joint-state prefixing accepts best-effort sensor publishers, checks dimensions/finite values and avoids repeated prefixes. It preserves positions, velocities, efforts and original timestamps.
- Removed the periodic `joint_state_publisher` aggregation from hardware launch. Both measured partial joint streams now feed the namespaced `joint_states` topic directly; `robot_state_publisher` handles partial updates. This avoids refreshing old measurements and fabricating default joint samples.
- Fixed empty/root namespace handling and static-TF relay retention: late subscribers receive the complete accumulated static transform set, not only the last incoming batch.
- ArUco localization validates IDs/pose lengths, finite pose values, quaternions, frames and observation age. Published robot TF retains the actual image timestamp and optical frame. It does not refresh stale/missing detections.
- Removed the uninitialized IMU counter/zero quaternion path and the assumption that accelerometer tilt determines camera translation and yaw. `nuc_extrinsics.yaml` defaults to `publish_camera_tf: false`; supply measured extrinsics or one external TF authority. Never enable two world-to-camera authorities or competing camera/odometry parents for the same base.
- Azure launch parameters are explicitly typed, including serial strings with leading zeros. Validate unsupported 30 FPS modes, time offsets, prefixes and wired-sync configuration. Implement the previously unused `required` process-exit behavior.
- Defaults are 15 FPS, NFOV_UNBINNED, 1080P color, 100 Hz IMU and no generated point clouds. Three-camera serials, FPS, sync modes, depth offsets and fixed exposure are configurable. Wired sync requires explicit serials/fixed exposure; host clock synchronization is still separate.
- Overhead ArUco defaults to RGB PnP rather than accepting a depth ROI median as a universally correct 3D tag center. Validate accuracy physically, especially planar-pose ambiguity and oblique views.
- New `human_hand_measurements` uses marker IDs (`body_id*100 + joint_id`), not array order. Publishes selected SDK hand joints 8/15 as timestamped `PoseWithCovarianceStamped`, rejects stale/duplicate/invalid samples and reports missing hands. Default `body_id=-1` publishes nothing. Body IDs are camera-local and may change on reacquisition.
- The hand covariance default is a configurable 5 cm standard-deviation assumption, **not measured accuracy or Azure SDK confidence**. Orientation is explicitly unobserved. These are hand locations, not calibrated fabric-corner contact points, and no hidden-hand prediction is presented as a measurement.

### Dependencies, packaging and deployment

`packages.repos` now pins:

| Dependency | Commit |
|---|---|
| Azure_Kinect_ROS_Driver | `0eaaa5645c1f1307c5b580ae305eb323a57807d2` |
| kinova-ros2 | `64c404b61e2897bddbe7e43a0442a1332852370c` |
| ros2-aruco-pose-estimation | `e43ea7ae8a99fcfecb77dfd4e7317b33df30790c` |

The associated files in `patches/` are required; merely pinning these upstream commits does not include the fixes. `scripts/apply_dependency_patches.py` checks exact revisions, validates all pending patches before mutation, recognizes already-applied patches, and never resets user changes.

- Azure patch: declare the previously ignored `depth_unit`; expose depth/color offset and fixed exposure; remove duplicate owning `shared_ptr(this)` construction; use normal volatile sensor-data image publishers instead of latched images.
- Kinova patch: initialize the velocity-conversion member rather than a shadowing local; stop incorrectly converting torque moments as angular velocities; validate publication rates; add a driver-side velocity-stream timeout that stops the API and requires explicit restart; fix gripper radians-to-firmware-count conversion and an uninitialized result pointer; remove invalid printf calls on ROS message objects; fix an assignment in a mouse condition and undefined actuator indexing. The vendor pose demo now requires an explicit target/motion opt-in, normalizes its quaternion and uses the ROS2 asynchronous action/result API with bounded waits instead of nonexistent ROS1-style client methods. Helper joint-state publication is configured at 50 Hz. Firmware-dependent velocity scaling/sign conventions still require measurement.
- ArUco patch: OpenCV 4.6/4.7+ detector compatibility, SciPy quaternion conversion and explicit Python dependencies, compatible image QoS, preserved annotated-image headers, explicit invalid-dictionary/depth-size checks, supported depth encodings, vectorized valid-depth ROI, and PnP/no-depth failure handling. The optional depth estimate remains an approximation, so RGB PnP is the default localization path.
- Added missing runtime/test dependencies, fixed include installation layout, installed description `control/` resources, added the missing action-server launch argument and corrected the hardware MoveIt gripper action type. Mock ros2_control demo configuration stays separate.
- Added C++17/warnings for local executable packages and regression tests. CI now triggers on configuration/scripts/dependencies as well as source; applies patches, builds, sources the install and runs behavioral tests. Checkout is pinned. The base container still uses `latest`; record its digest when qualifying a deployment. CI itself was not run remotely in this session.
- Replaced unattended `vcs pull`/root rebuild logic with explicit maintenance scripts that refuse an active runtime, use fast-forward-only updates and reject mismatched dependency revisions. CD service templates are maintenance-only, not enabled at boot; launch templates no longer rely on an automatic update first. Existing installed services are unchanged and must be migrated manually.
- Replace the vendor-only/world-writable Rokubi udev alias with a per-serial, `dialout`/0660 template. It is intentionally commented until the correct serial is entered. SDK downloads now fail on HTTP errors, use HTTPS and clean temporary directories. The vendor SDK packages remain old distributions; runtime compatibility must be qualified on the actual GPU/OS.
- Add `scripts/record_fabric_baseline.sh` with MCAP and explicit `/tf_static` QoS, updated setup/teleoperation documentation, and Python artifact ignores.

## Verification performed

All 23 packages build on the installed Jazzy environment: 15 local packages and eight imported dependency packages, including the patched Kinova driver, vendor demo and Azure sensor driver. Azure SDK 1.4.1 was extracted under `/tmp` for this check; it was not installed system-wide. **The optional Azure body-tracking SDK/GPU path was not compiled or run locally.**

Checks performed:

- Platform pytest suite: **49 passed** (15.64 seconds on this host); launch construction, URDF/MoveIt expansion, mesh references, valid package/YAML/Python configuration, pure calibration/guard tests, ROS tests using fake messages/pseudo-terminals, offscreen Qt GUI, a synthetic image-to-ArUco-pose pipeline on stock OpenCV 4.6, and the vendor pose demo against a simulated ROS2 action server.
- Package tests via `colcon test`/`colcon test-result`: **23 reported tests, zero errors/failures, two pre-existing copyright-template skips** (includes test-runner aggregates). The C++ serial suite covers standard CRC, fragmented packets, corruption/resynchronization and consecutive packets.
- ROS integration checks: best-effort joint delivery with unchanged stamps, static-TF late join, invalid namespace rejection, Arduino stream loss, no stale/TF-less F/T publication, ArUco stamp/frame preservation, guard loss/re-arm, marker-ID hand selection, admittance TF/disable/sensor-loss stops, and pose-demo motion opt-in/quaternion normalization/action completion. The ArUco test waits for a result within a bounded discovery timeout rather than assuming a fixed startup delay.
- All three dependency patches applied to fresh exact-revision checkouts; a second application correctly recognized each as already applied.
- `bash -n` for shell scripts and `git diff --check` passed. No hardware launch was run. Documentation text was updated; the Docusaurus site build was not run because Node.js is absent here.

The ROS tests force localhost-only discovery on domain 197. The execution sandbox initially blocked UDP sockets; the same tests were rerun with permission for local sockets and passed. No pass result is inferred from the sandbox-blocked runs. Test-driven fixes included a diagnostic-message byte type and a collision with an rclpy property in the new hand adapter.

## Build and deployment procedure

Use a clean workspace for the first rollout; preserve the previous qualified install for rollback. A conventional nested layout is also supported: import dependencies into the workspace's `src` alongside this repository, passing that directory to the patch script.

```bash
source /opt/ros/jazzy/setup.bash
# From a repository-as-workspace checkout:
vcs import src < packages.repos
python3 scripts/apply_dependency_patches.py --workspace-src src
rosdep install --from-paths src --ignore-src --skip-keys python3-pyside6 -y
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install PySide6==6.8.3
colcon build
source install/setup.bash
colcon test --packages-select rokubi_force_torque_publisher force_torque_calibration oarbot_runtime tablet_gui
colcon test-result --verbose
bash scripts/check_platform.sh
```

Install the actual Azure sensor/body SDK and GPU runtime on the helper computers using the existing SDK instructions; the local compile-only SDK extraction is not a robot installation. Dependency branches should not be pulled after patching. For an upgrade, qualify a fresh checkout, update the pin and patch together, and rerun the tests.

Before launching hardware:

1. Install a sensor-specific serial rule or pass `ft_serial_port:=/dev/serial/by-id/...`. Assign explicit Azure serials. Verify the tablet serial device and `dialout` membership.
2. Measure and save the overhead extrinsics; verify wrist camera and gripper transforms and one authority for every TF child. Select the intended `body_id` independently on each wrist camera.
3. Confirm domain 0 for helpers/tablet/NUC and the installed Dingo bridge domains (Silver 1, Blue 2). Match the actual network interface in CycloneDDS configuration. Configure host clock synchronization and measure it.
4. Run the hardware checklist below. Use one robot at a time for initial axis/stop tests.
5. Install the updated scripts/service templates only in a maintenance window. Disable previously enabled automatic `*_cd.service` units; replacing a template does not remove existing enablement symlinks. No service installation/restart was performed by this audit.

Example argument surface, **after** acceptance and calibration:

```bash
ros2 launch oarbot_launch oarbot_blue_helper_laptop.launch.py \
  sensor_sn:=ACTUAL_BLUE_CAMERA_SERIAL \
  ft_serial_port:=/dev/serial/by-id/ACTUAL_SENSOR \
  body_id:=SELECTED_BODY_ID
ros2 launch oarbot_launch nuc.launch.py extrinsics_file:=/absolute/path/calibrated_nuc.yaml
```

For wired sync, additionally set one master/two subordinates, equal FPS, measured exposure and suitable offsets. Do not copy arbitrary offsets without checking interference and timestamps. `setup_mode:=true` bypasses guarded velocity operation and is only for supervised position-action commissioning. Returning to normal operation requires restarting in the default mode. A driver watchdog stop requires an explicit driver start service after the cause has been resolved; it does not silently resume when packets return.

## Hardware and manual acceptance gates

Record the hardware/firmware serials, dependency SHAs, patch hashes, container/OS/GPU versions, calibration files and bags for each run. Suggested quantitative targets below are commissioning targets, not demonstrated results.

| ID | Test | Required observation before proceeding |
|---|---|---|
| H1 | Physical E-stop and Dingo/arm stop wiring | Verify which actuators each physical stop actually inhibits. Software Bool topics and the guard are not the physical E-stop. Verify restart requires the intended deliberate operator action. |
| H2 | Command loss and restart | With a small supervised motion, release deadman, assert stop, unplug Arduino/SpaceMouse, stop GUI/admittance/guard processes, interrupt Wi-Fi and restart components. Measure command cessation and physical stopping distance. Verify no automatic restart with the deadman held. Confirm the driver's API-stop watchdog and Dingo's embedded timeout when the helper itself loses power. |
| H3 | Command axes and units | Test each translational/rotational axis independently on Blue/Silver at reduced speeds. Check Kinova API angular axes, the Silver mount transform and base-frame planar convention. Compare finite-differenced encoder positions against reported joint velocities; the vendor firmware conversion includes historical scale/sign behavior. Only then enable angular admittance. |
| H4 | Position-action setup | In supervised setup mode test home, fingers and small joint trajectories: bounds, units, rejection, cancellation, preemption, stop, timeout and driver restart. Validate the vendor action adapters under these cases before enabling automated planning. The adapters still contain legacy threading/API assumptions not proven by compilation. Never mix their raw APIs with the guarded velocity mode. |
| H5 | F/T axes, origin and load | Apply known loads in all directions; verify sign, N/Nm, moment origin, Blue/Silver rotations, saturation/error status, sample rate and latency. Confirm the modeled sensor wrench origin against the manufacturer's reference. Verify unloaded residual over diverse static orientations; set a measured tolerance, e.g. initially <1 N if repeatable. Recalibrate with the actual camera/gripper/cable routing and no human/sheet load. |
| H6 | F/T failure and dynamics | Unplug sensor and hide/drop joint TF: no stale calibrated output, diagnostics fault, admittance stops. Compare loaded/empty motion at increasing speed; do not interpret payload inertia or cable loads as sheet forces. Characterize bias/temperature drift and device tick-to-host timing before faster handling. |
| H7 | Camera intrinsics/extrinsics | Validate depth scale, RGB-depth registration and lens model on a known target. Measure overhead six-DOF pose and both wrist hand–eye transforms over several arm configurations. Aim for approximately 1–2 cm cross-view alignment initially so calibration does not consume the entire 5 cm fabric budget. Check transforms at acquisition times, not only in a static RViz scene. |
| H8 | Localization and occlusion | Validate ArUco marker size/mount, pose ambiguity and base pose against ground truth. Occlude each base marker: no newly stamped pose is generated; consumers must detect stale localization. For moving bases under persistent occlusion, add qualified odometry/localization fusion before relying on world-frame fabric estimates. |
| H9 | Three-camera synchronization | Confirm cabling, serial selection, equal FPS, exposure, sync master/subordinate status and depth-emitter offsets. Measure actual RGB/depth capture skew, inter-host clock error, jitter and timestamps under load. Initial targets: <5 ms host-clock error and <10 ms inter-view skew for slow handling; derive tighter limits from speed × timing error later. |
| H10 | Azure body SDK and person identity | Qualify body SDK/GPU startup and sustained throughput on each actual helper. Check selected person, hand IDs, hand-vs-grip offset, occlusion, crossing people, body-ID changes and reacquisition. Missing observations must remain missing. Calibrate covariance empirically; the adapter does not provide SDK joint confidence or cross-camera association. |
| H11 | Dry carbon-fiber observability | Record the real ply at several angles/distances/lighting conditions, flat and wrinkled. Quantify valid-depth fraction, holes and repeatability; use RGB/other views where ToF is unreliable. Test wrist/gripper/human occlusion. No camera mode alone guarantees 5 cm sheet accuracy. |
| H12 | Sustained computation and recording | Run all cameras, body tracking, TF, robot states and F/T for at least 30 minutes. Measure sensor rates, p95/p99 acquisition-to-consumer age, CPU/GPU load, thermal throttling, memory, queue sizes, USB resets and DDS/bag drops. Full-resolution raw images from three cameras exceed ordinary Wi-Fi budgets; prefer processing/recording near each camera or provision sufficient wired bandwidth. |
| H13 | GUI, services and power cycling | Test touchscreen/SpaceMouse semantics, disabled controls, closing/reopening GUI, logs and systemd restart behavior on the actual tablet/helpers. Check venv interpreter selection, device aliases after reboot, explicit maintenance updates and rollback. The offscreen test does not validate touchscreen ergonomics or the tablet USB firmware. |
| H14 | Future fabric observer acceptance | Before any observer-driven correction, replay bags offline, use independent shape truth and evaluate position/velocity error, timestamp age, visible-vs-predicted regions and uncertainty coverage. Define “5 cm” explicitly (e.g. 95th-percentile visible-surface error), plus an independent occlusion/prediction test. Then connect the simulator in read-only/shadow mode before force/state feedback. |
| H15 | Physical robot model | Check link/mount dimensions, tool TCP, joint limits, collision meshes and SRDF disabled-collision pairs against the real assembly. The URDF resource/tree checks do not validate physical dimensions, inertias or collision avoidance. Qualify payload/inertial parameters before dynamic simulation or automated planning. |

## Interfaces for the next implementation phase

For each robot (`oarbot_blue` / `oarbot_silver`):

- `/<robot>/joint_states`: measured partial joint samples with original source stamps. Do not assume every message contains all joints, or that zero finger velocity/effort is a measured value; the vendor supplies no complete finger dynamic state.
- `/<robot>/force_torque_raw` and `force_torque_calibrated`: stamped Wrench messages, raw and calibrated measurement frames, moments about the modeled sensor origin. To change the reference point, use the full wrench transform including the moment-arm cross product.
- `/<robot>/human/{left_hand,right_hand}/pose`: selected per-camera hand position observations in the original camera frame. Transform at the header time; estimate grip offsets, fuse identity/views and estimate velocities in the future observer.
- `/<robot>/azure_kinect/{rgb,depth,depth_to_rgb}/image_raw` and camera info; equivalent overhead topics under `/nuc`. `16UC1` depth is millimeters; `32FC1` depth is meters. Invalid depth remains invalid.
- `/tf`, `/tf_static`, ArUco observations and per-robot diagnostics. Check both observation age and TF availability before using state.

The next phase should implement the mesh observer and simulator bridge described in [fabric-state-estimation-research.md](fabric-state-estimation-research.md), with independent state, parameter and residual-force update paths. It must explicitly represent observed vs inferred vertices, uncertainty and prediction horizon. This audit does not make hidden fabric geometry observable or supply accelerations from noisy position differences.

Conventions followed: [REP-103 units/frames](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP-105 frame authority](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst), and [ROS2 Jazzy QoS compatibility](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst). The tests and explicit hardware gates distinguish source correctness from physical calibration and runtime performance.
