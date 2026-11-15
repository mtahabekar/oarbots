# fabric_cell

Launch files and configuration templates for running the fabric estimator, the twin and the
sensitivity CBF on the real OARBot cell. Everything under `config/` is a template: copy the
directory, measure and tune the values for the cell, and pass it as `cell_dir:=/abs/path`.
`docs/fabric-hardware-test-guide.md` in the repository has the bring-up order and the checks.

| Launch file | Run on | Starts |
| --- | --- | --- |
| `robot_host.launch.py robot:=blue|silver` | each helper laptop | wrist-camera perception, fabric CBF bridge |
| `camera_host.launch.py camera:=overhead` | the NUC | overhead-camera perception (sheet and glove tags) |
| `compute.launch.py` | the compute machine | calibration TF, hand fusion, estimator, twin + CBF, parameter bank, RViz |
