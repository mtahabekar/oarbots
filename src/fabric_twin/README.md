# fabric_twin

A simulated copy of the sheet that runs beside the real cell. Its four grips follow the measured
robot and glove grips, the estimator corrects it, and the sensitivity CBF runs on it against a
simulated obstacle. See `docs/fabric-twin.md` in the repository for the design, the commands and
the measured results.

Extended identification motions and comparison procedure are documented in
`../fabric_sim_cell/README.md` (Extended material-identification warm-up).
Run CPU trajectory checks with:

```bash
PYTHONPATH=src/fabric_twin ~/venvs/fabric_ros/bin/python -m pytest -q src/fabric_twin/test/test_warmup.py
```
