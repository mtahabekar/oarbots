# Kinova status at the grasp pose (2026-11-18, 22:10)

Read-only probe (`kstatus.cpp`: Get* calls only), run on each helper with that robot's fabric session
stopped (the arm holds its pose) and restarted afterwards. Both arms at the grasp pose reached by
`grasp_joints.py` (the simulation's `grasp_arm` joints).

| | Blue | Silver |
| --- | --- | --- |
| CartesianFaultState | **1** | **1** |
| ControlEnableStatus / ControlActiveModule / ControlFrameType | 1 / 1 / 0 | 1 / 1 / 1 |
| Singularity vector (counts, distances, repulsion) | all 0 | all 0 |
| Protection zones | 0 | 0 |
| Cartesian position (m, rad) | 0.4813 -0.0032 0.3929, 2.2539 0.6632 -0.4368 | -0.4813 -0.0056 0.3933, 2.2511 -0.6584 -2.7115 |
| Actuator angles (deg) | 330.61 167.86 272.21 86.66 -82.16 106.84 | 29.12 192.15 87.90 273.34 442.30 253.16 |

On 2026-11-17 at Home both arms read CartesianFaultState 0 (`2026-11-17_kinova_config/`). At Home the
elbows are at 57 deg (Blue) and 303 deg (Silver) and the wrists (joint 5) at +83 / -83 deg; the grasp
joints put both on the other branch (elbow past straight, joint 5 through zero). With the fault set, the
firmware ignores Cartesian velocity: the inward moves at 17:19-17:38 forwarded ~100 Hz of 3 cm/s commands
with zero joint change. Joint velocity works at this pose (both approach moves ended within 0.9 deg).
Decision (user, 22:12): the fabric following uses joint velocity (the bridge converts the twin's
world-frame velocity with the robot model) instead of Kinova's Cartesian control.

Build and run on a helper (the fabric session stopped, so the USB is free):

```bash
g++ -O1 -o kstatus kstatus.cpp -I ~/fabric_ws/src/kinova-ros2/kinova_driver/include/kinova -ldl
LD_LIBRARY_PATH=~/fabric_ws/src/kinova-ros2/kinova_driver/lib/x86_64-linux-gnu ./kstatus
```
