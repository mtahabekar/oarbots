# sim_eval: does each module of the perception–simulation–control loop do its job?

A simulation study of the fabric pipeline in which the Isaac Sim cell plays the part of the real
system. The plant's sheet has a **hidden material** that the twin does not know; every module sees
only what it would see on hardware (rendered RGB-D, tagged gloves, robot joint states). Plant truth
is used only for scoring.

```
Isaac Sim plant (hidden bending/shear scales θ*)
   │ rendered cameras, robot joint states, glove tags
   ▼
perception + hand tracking + state estimator ──► observed sheet, measured grips
   ▼                                                  │
fabric twin (+ correction) + CBF + stretch guard      │ depth, twin state, grip history
   │ follower velocity commands                       ▼
   ▼                                           parameter bank ──► fitted θ̂ ──► twin, CBF model
Isaac Sim robot arms
```

## Design principle: isolate each module

The closed loop couples everything, so a closed-loop number mixes every module's errors. The study
therefore has two kinds of tests.

1. **Recorded-input tests (S1 → S2, S3).** The plant runs once per dataset with the robots driven
   only by inputs no module under test can change: the warm-up script and formation-following of
   the measured hands. The CBF is unfiltered (`controller: none`), so the twin's model never moves
   the robots. Every input a module consumes is recorded (grips the twin used, every gated depth
   point in the world frame, the estimator's analyses, the twin's states, the plant truth). Each
   module is then replayed offline with **one switch changed at a time and everything else
   identical**, and its output is scored against plant truth. Differences between conditions are
   paired: same dataset, same inputs.
2. **Closed-loop tests (S4, S5).** The twin's commands drive the robots past the obstacle. These
   measure what the modules are for (clearance, pull, task) and are repeated, because the
   asynchronous stack makes runs differ.

Truth inputs replace one perceived input at a time ("oracle" conditions). That separates a
module's own error from errors it inherits: identification with true sheet state, true grips and
the true surface tests the warm-up and the bank alone; adding each perceived input alone shows
which one costs accuracy.

## Plants and hidden materials

| Plant structure | Mesh, solver, grip patch | Why |
| --- | --- | --- |
| `matched` | 28 × 28 cells (3 cm), 8 substeps, 6 PD / 3 PCG iterations, 2 × 2 pinned particles (3 cm) | Exactly the twin's model class. θ* is the true answer in the twin's coordinates, so identification error can be measured. A twin with θ* and true grips, started from the true state, stays within 0.08 mm of the plant over 2 s. |
| `fine` | 56 × 56 cells (1.5 cm), 8 / 8 iterations, 3 × 3 pinned particles (3 cm) | The default rehearsal plant: a structurally different sheet, as reality will be. No exact θ* exists; prediction on held-out motion is the test. |

| Material | bend × | shear × | Meaning |
| --- | --- | --- | --- |
| A | 1 | 1 | control: the twin's nominal coefficients are right |
| B | 3 | 0.4 | stiffer in bending, softer in shear |
| C | 1/3 | 2.5 | softer in bending, stiffer in shear |

Scales multiply the twin's nominal Style3D values (edge `ke` 0.002 / 0.001 / 0.0005, triangle shear
`ke` 6000) in the plant only (`--edge-ke-*`, `--tri-ke-shear`). Datasets: matched A, B, C × 3
repeats; fine A × 2, fine B × 3. Repeats use identical settings: they differ only in the timing of
the asynchronous stack (rendering, ROS delivery, estimator latency).

## S1: identification datasets (`scripts/run_id.sh STRUCTURE MATERIAL REPEAT`)

One 147 s plant run (`config/twin_id.yaml`, hand script `id_script.npz`):

| Time (s) | Hands | Robots |
| --- | --- | --- |
| 0 – 15 | still (2 cm gather at 10.5 s) | attach, then formation following |
| 15 – 93.3 | still | extended warm-up: insets 2 and 4 cm, 3 cm transverse compression, 5 cm lift of each robot, ±3 cm lateral shift, return; 13 holds of 5 s at 0.03 m/s |
| 97.5 – 140.4 | held-out validation motion: lower both 15 cm, lower the left 12 cm, shift +10 cm, swap the low hand, shift back, raise; ≤ 0.06 m/s | formation following (stretch guard on, CBF unfiltered) |

The validation motion is never used for fitting. It tilts and twists the sheet, where bending and
shear matter, and stays above the table.

Recorded: `truth.npz` (plant particles, 60 Hz), `twin.npz` (twin states 15 Hz, the four grips it
was stepped with at 60 Hz, commands), `twin.depth.npz` (every gated depth point, world frame, as
the parameter bank receives it), `estimate.npz` (estimator analyses with covariance and
visibility), `estimate.hands.npy` (glove filter output), `bag/` (camera observations, glove
detections, TF, joint states, for estimator replays), `manifest.json` (condition, θ*, commit).

## S2: material identification (`analysis/id_fit.py`, offline)

The production bank (`fabric_twin.bank.Bank.calibrate`, the search the live bank runs) fits
(bend, shear) to the warm-up holds of each dataset. Inputs per case, each from truth or from the
live system:

| Level | Start state | Grips | Surface points | Question |
| --- | --- | --- | --- | --- |
| L0 | truth | truth | truth (plant particles) | Is the warm-up informative, and does the search find θ*? |
| Dp | truth | truth | **rendered depth** | Cost of depth noise and coverage |
| Gm | truth | **measured** | truth | Cost of grip (TF + glove) error |
| St | **corrected twin** | truth | truth | Cost of starting from the twin instead of the true sheet |
| L3 | twin | measured | depth | The live configuration |

Each level is run with the 13 extended holds and with the 4-hold baseline subset (settle, 2 cm,
4 cm, final return) of the same run, so the two warm-ups are compared on identical data. Dp, Gm and
St run on repeat 1 of the mismatched groups (matched B, matched C, fine B). Cost landscapes
J(bend, shear) on a 5 × 5 log grid (1/4 … 4) show identifiability for L0 and L3 on the same three.

Metrics: log error log(θ̂/θ*) per coefficient, repeat-to-repeat spread, cost, the bank's
sensitivity check, fit time. The bank's search returns values on a 2^(1/4) grid, so its
resolution floor is about 5–6 % in log error for these θ*.

## S3: twin and correction (`analysis/twin_replay.py`, offline)

The twin is replayed over each dataset (12 s → end) from the plant's true state at 12 s, driven by
the recorded grips. Switches:

| Condition | Correction | Coefficients | Grips | Isolates |
| --- | --- | --- | --- | --- |
| off_nominal (B1) | off | nominal | measured | grip-driven simulation, the baseline |
| off_fitted (B2) | off | S2 L3 fit of this dataset | measured | value of calibration alone |
| off_true | off | θ* | measured | value of perfect calibration |
| est_nominal (P0) | estimator | nominal | measured | value of visual correction alone |
| est_fitted (P) | estimator | fitted | measured | the full loop |
| oracle_nominal | truth at the estimator's vertices and times | nominal | measured | correction with a perfect observer |
| off/est_nominal_truthgrips | off / estimator | nominal | truth | hand-tracking error removed |
| est_nominal_gain{0.25,4} | estimator, gains × k | nominal | measured | sensitivity to correction gain |

Metrics: per-state mean and p95 particle error and the lowest-5 % height error (what clearance
depends on), in the warm-up and in the held-out validation motion; open-loop forecast error at
0.1, 0.33, 0.5 and 1 s from origins every 1 s of the validation motion (clone the twin, free-run
with the recorded future grips, no correction).

## S4: closed-loop carry (`scripts/run_closed.sh CONDITION MATERIAL REPEAT`)

Fine plant with hidden material B, the carry script used in the hardware rehearsals (hands lower
0.55 m past the obstacle, ±8 cm, rise). Conditions B0 (no clearance CBF), B1 (CBF on the
grip-driven twin, nominal), B2 (grip-driven, fitted θ̂ from S2), P0 (corrected, nominal), P
(corrected, fitted), G0 (P without the stretch guard); 3 repeats each. θ̂ is the median S2 L3 fit
of the fine-B datasets, so the closed loop does not depend on one live calibration.

Metrics (`analysis/score_closed.py`): true minimum clearance from the box and from the CBF's
capsule, time and depth inside the 4 cm margin, contact time, true robot-grip pull (plant
mechanics; raw and 0.1 s low-pass), formation error, CBF and guard status counts, zero-command
fraction.

## S1b: observer isolation (`scripts/run_obs.sh DATASET CONDITION`)

The recorded bag of repeat 1 of each group is replayed through the estimator stack (no Isaac Sim)
in three conditions: `vision` (as live), `novision` (`config/estimator_novision.yaml`: no camera
factors, so every analysis returns the prior, i.e. the ensemble driven by grips alone) and
`oracle_vision` (plant mocap hands). Scored by phase against plant truth (`summarize.py s1b`).

## S2b–d: identification remedies (offline, repeat 1)

- `id_fit.py --relax 4.5`: longer free runs per case.
- `iterate_fit.py [--relaxation 0.5]`: refit from a twin replayed with the latest estimate (plain or
  half-step).
- `long_fit.py`: output-error fit, one free run through the whole warm-up, scored at every hold
  against each capture at its own time.
- `adapt_trace.py`: the bank's online `adapt` every 2 s from nominal coefficients (all datasets).

## S5: full live loop (`scripts/run_full.sh MATERIAL REPEAT`)

Fine plant, hidden material B. The live twin runs the extended warm-up at 15 s, the live bank
calibrates beside Isaac Sim and releases to the twin and its CBF model, online adaptation runs every
10 s, and the carry starts at 480 s (hand script `full_script.npz`). Scored like S4 over the carry
window, plus a whole-run timeline (`analysis/s5_timeline.py`).

## Reproduce

```bash
# S1 (Isaac Sim; one at a time)
experiments/sim_eval/scripts/queue.sh run_id.sh "matched A 1" "matched B 1" ...
# S2, S3 (offline, parallel GPU jobs)
cd experiments/sim_eval/analysis
source /opt/ros/jazzy/setup.bash && source ~/ros2_colcon_ws/install/setup.bash
PYTHONPATH=.:../../../src/fabric_twin:../../../src/fabric_sim_cell:$PYTHONPATH ~/venvs/fabric_ros/bin/python run_offline.py s2
PYTHONPATH=... ~/venvs/fabric_ros/bin/python run_offline.py s3
# S4
experiments/sim_eval/scripts/queue.sh run_closed.sh "B0 B 1" "P B 1 <bend> <shear>" ...
```

Run data live in `$SIM_EVAL_DATA` (default `/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval`);
results (JSON/NPZ) in `results/`, figures in `figures/`.
