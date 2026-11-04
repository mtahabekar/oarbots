**Fabric perception implementation**

This implements the Perception block of [the research architecture](fabric-state-estimation-research.md), with tagged, single-colour gloves and prescribed corner holding. It produces observations for both the online causal observer (Tier B) and offline reference reconstruction (Tier A). It does not yet implement either mesh estimator, simulator identification, or twin assimilation.

The fabric starts attached to both grippers. Each human glove has a different ArUco ID; colour alone never establishes hand identity. Each hand is assumed to hold its configured material corner whenever its grip position is observed. `holding_state=HOLDING` and `holding_is_assumed=true` describe the task assumption, not a contact classification. There is no grasp planner, release detector, slip detector or MANO dependency in this version. A hidden tag yields missing data, not a fabricated corner measurement.

**Packages and data flow**

```text
RGB + registered depth + RGB CameraInfo + optional robot mask
  -> fabric_perception/camera (one on each camera host)
       -> fabric/observations: contours, depth samples, tracks, optional UV priors
       -> fabric/gloves: per-view identified grip position + covariance
            -> human_hand_tracking/fusion (one on estimator host, using capture-time TF)
                 -> /fabric/hand_states: filtered world positions/velocities + covariance
```

`fabric_state_msgs` contains the ROS contracts. All camera outputs retain the RGB capture stamp and optical frame. The depth stamp is retained separately. Fusion output time is the newest capture in a group; each accepted source camera and capture stamp is retained. No full-resolution image is sent to the estimator unless debug images are enabled or raw recording is requested. Tracks are keyed by camera, process lifetime and track ID. They have no material correspondence until associated by a later estimator.

**Implemented front ends**

- Sheet segmentation: a configurable dark, low-saturation HSV baseline within a workspace ROI, or a trained four-class ONNX segmenter. The baseline requires a contrasting background and careful ROI; it cannot distinguish arbitrary dark robots/backgrounds. Keep all sufficiently large disconnected visible components. Exclude dilated glove masks and optional robot masks. No hole filling across hands or folds.
- Gloves: HSV support plus unique ArUco IDs, subpixel tag corners, metric planar PnP, reprojection gating, planar ambiguity checks, and a calibrated tag-to-grip offset. First-order corner-noise covariance includes an additional position floor. Both gloves may have the same colour. Tags supply identity even when hands cross.
- Depth: consume only depth registered to RGB. Convert `16UC1` millimetres or `32FC1` metres; reject invalid/range-exceeding values, discontinuities, neighbours of holes and silhouette/occlusion margins. Unproject with the RGB distortion model. Publish decimated samples and axial variance. **This is quality gating of raw depth, not learned depth refinement.** RGB contours, tracks and UV priors remain available through complete ToF dropout.
- Boundaries: outer contours and holes in raw RGB pixels, sharing a bounded sample budget (largest contours first). Confidence is zero near hands, robots, ROI/image edges; those edges must not become material-boundary factors. Contours use polygon approximation to 0.5 pixels, not a claimed metrology-grade subpixel detector.
- Tracks: bounded KLT features with forward/backward checks, boundary exclusion and identity retention. Reset after large capture gaps or calibration changes. This is a sparse baseline, not CoTracker/DELTA; repetitive weave and specular highlights can still defeat it. No fabric velocity is inferred by finite differences.
- UV: optional ONNX per-pixel material-UV prior. Without trained weights, the array is empty and `uv_source=unavailable`. No rectangular image-to-UV shortcut and no inferred hidden surface is published. UV sampling is independent of depth validity.
- Multi-view hands: bounded timestamp reordering; sample-time TF for moving wrist cameras; covariance rotation and extrinsic uncertainty; maximum mutually consistent subset of views; information fusion with a floor that cannot be averaged away; joint position/velocity/acceleration Kalman filtering with innovation gating. Two conflicting views without a unique consensus yield no observation. Position/velocity covariance is published jointly. Missing hands are omitted; stopped streams produce stale diagnostics and are never refreshed by a publication timer.

Depth noise, tag noise, heuristic confidence and UV covariance are configuration assumptions requiring held-out calibration. They are not validated uncertainty coverage or measured accuracy. Learned model sources carry SHA-256 hashes. Axial depth quality and image tracking quality remain separate; neither supplies tangential material confidence.

**Build and run**

From the colcon workspace (after installing dependencies with `rosdep`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fabric_state_msgs human_hand_tracking fabric_perception
source install/setup.bash
```

Copy `src/fabric_perception/config/camera.yaml` for each camera and replace its example values. Set the glove hue bounds, true tag size, calibrated offsets, workspace ROI, ToF working range and measured noise. Zero grip offsets are examples only: an attached tag is not automatically the fabric pinch point. A rigid cuff/tag fixture is preferable to a tag that bends with a glove. The marker convention is OpenCV `SOLVEPNP_IPPE_SQUARE`: origin at tag centre, x right, y up in the printed tag, z out of its face. Measure the outer black square without the white margin. Defaults use IDs 20/21 in `DICT_4X4_50`, separate from robot IDs 0/4.

Launch alongside the existing camera driver on each helper laptop:

```bash
ros2 launch fabric_perception camera.launch.py camera_namespace:=/oarbot_blue/azure_kinect camera_id:=blue_wrist params_file:=/path/to/blue.yaml
ros2 launch fabric_perception camera.launch.py camera_namespace:=/oarbot_silver/azure_kinect camera_id:=silver_wrist params_file:=/path/to/silver.yaml
```

Optionally launch a third copy for `/nuc`, `camera_id:=overhead`, with its own config. Run once on the estimator host:

```bash
ros2 launch fabric_perception fusion.launch.py params_file:=/path/to/fusion.yaml
```

Set `world_frame` to the cell's calibrated common frame and configure `observation_topics` to the cameras actually running. The human corner UV convention in the example is left `(0,1)`, right `(1,1)`, robot corners at `v=0`; change it to match the persistent material mesh. This does not assign UV to the visible fabric surface. Sheet dimensions/mass are unnecessary for this front end and remain required inputs to the later dynamics estimators.

Inputs relative to the camera namespace:

| Topic | Contract |
|---|---|
| `rgb/image_raw` | RGB/BGR/BGRA image convertible by cv_bridge; nonzero source stamp |
| `rgb/camera_info` | RGB raw intrinsics; full image, `plumb_bob` or `rational_polynomial`; may be cached |
| `depth_to_rgb/image_raw` | Same resolution and optical frame as RGB, independently stamped registered depth |
| `fabric/robot_mask` | Optional `mono8`, nonzero robot pixels, exact RGB stamp/frame/resolution |

Raw IR is recorded for future camera-specific depth training, but is not fused into this baseline because it is in the depth camera's geometry. An exact TF/URDF robot renderer is not included: supply the exclusion mask and set `require_robot_mask=true`, use the ONNX robot class, or constrain the baseline ROI. An estimator-rendered fabric mask is deliberately not fed back as an independent observation.

Both image and observation subscriptions use sensor-data QoS (best effort). RGB/depth queues are bounded, default pairing tolerance is 15 ms and freshness limit 250 ms. When a robot mask is required, up to five paired captures wait for their exact-capture mask, subject to the same freshness limit. Hardware sync and clock calibration remain necessary: approximate pairing does not synchronize devices. Out-of-order, duplicate, stale and future captures are rejected. Missing TF is never replaced with latest TF. `use_sim_time:=true` on both launch commands supports recorded capture times with `ros2 bag play --clock`; restart the nodes when seeking backwards or looping a bag.

Camera outputs under `fabric/`: `observations`, `gloves`, `diagnostics`, and optional `labels`/`depth_valid` images. Debug semantic labels are background=0, sheet=1, glove/hand=2, robot=3, uncertain=4. Debug depth validity is 0/255. Full raw data plus observations can be recorded with `scripts/record_fabric_baseline.sh` (MCAP). It also records optional exclusion/debug masks when published. Archive the camera/fusion YAML and model files with each bag to preserve calibration, thresholds and model provenance.

**Learned-model contracts**

Both OpenCV DNN adapters consume float32 `[1,3,H,W]`, RGB scaled to `[0,1]`, resized directly to `model_input_size` (width,height), with no additional normalization. Export the network with that preprocessing, and train with the same resizing. Outputs are resized to raw RGB resolution.

| Model | Output | Semantics |
|---|---|---|
| Segmentation | `[1,4,H,W]` finite logits | Classes background, sheet, human, robot. Softmax threshold rejects uncertain pixels. Set `segmentation_backend=onnx` and `segmentation_model=/path/to/model.onnx`. |
| UV | `[1,3,H,W]` values in `[0,1]` | Material u, material v, confidence. Set `uv_model=/path/to/uv.onnx`; confidence threshold gates predictions. Covariance is separately configured, pending calibration. |

Malformed outputs fail the frame with diagnostics. No weights are downloaded or implicitly trusted. Synthetic training, real fine-tuning, learned depth repair, dense long-range tracking and multi-hypothesis completion remain the subsequent learning stage described in the research. The current adapters implement inference, not those training pipelines.

**Verification and next integration**

Unit tests exercise masks/occlusions, depth holes and folds, distortion-aware projection, persistent tracks, rendered glove tags and grip offsets, covariance transforms, view conflicts, temporal rejection and dropout/reacquisition. ROS tests exercise synchronized images, sensor-data QoS, source stamps, empty depth, exact-capture masks, missing/stale observations, capture-time TF, and assumed holding. Run the isolated suite after building:

```bash
python3 -m pytest -q src/fabric_perception/test src/human_hand_tracking/test
```

These tests provide software verification; no physical carbon/glove accuracy or latency target has been demonstrated. Use the debug masks, valid-depth fraction, reprojection residuals, processing age and hand status for the research's stage-0 observability recordings. Validate real tag-to-grip and hand-eye calibration before using these boundary observations. Next, feed these raw observation contracts into Tier A and Tier B while keeping their shared material mesh as the state; do not fit a mesh here by nearest-neighbour attraction or accumulate a static TSDF.
