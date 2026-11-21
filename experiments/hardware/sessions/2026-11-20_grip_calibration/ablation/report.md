# Warm-up calibration and factor test

Warm-up /home/taha/ros2_colcon_ws/src/oarbots/experiments/hardware/sessions/2026-11-20_fabric_run/warmup_10_0.1_111137 25:49; held out: held out live_115658 40:120; held out live_120408 20:90; held out warmup_hardware_215321 10:100

Repeatability: baseline sd 0.14 mm over 4 runs; detection threshold 1.00 mm.

Detected: required mesh 14x14; bending best x0.1 (gain 0.14 mm, adopted False); shear best x1 (gain 0.00 mm, adopted False); with grip points shear x1, bending x1.

| Config | warm-up (fit) | held out live_115658 40:120 | held out live_120408 20:90 | held out warmup_hardware_215321 10:100 |
| --- | --- | --- | --- | --- |
| baseline | 27.3 (-22, 59) | 32.2 (-23, 88) | 46.0 (-64, 137) | 31.3 (+1, 123) |
| S | 27.3 (-22, 59) | 32.2 (-23, 88) | 46.0 (-64, 137) | 31.3 (+1, 123) |
| B | 27.3 (-22, 59) | 32.2 (-23, 88) | 46.0 (-64, 137) | 31.3 (+1, 123) |
| BS | 27.3 (-22, 59) | 32.2 (-23, 88) | 46.0 (-64, 137) | 31.3 (+1, 123) |
| G | 19.5 (+6, 45) | 27.5 (+4, 67) | 44.1 (-50, 134) | 29.5 (+5, 104) |
| GS | 19.5 (+6, 45) | 27.5 (+4, 67) | 44.1 (-50, 134) | 29.5 (+5, 104) |
| GB | 19.5 (+6, 45) | 27.5 (+4, 67) | 44.1 (-50, 134) | 29.5 (+5, 104) |
| GBS | 19.5 (+6, 45) | 27.5 (+4, 67) | 44.1 (-50, 134) | 29.5 (+5, 104) |
| R | 25.0 (-20, 52) | 31.2 (-17, 86) | 40.8 (-36, 132) | 28.5 (+3, 123) |
| RS | 25.0 (-20, 52) | 31.2 (-17, 86) | 40.8 (-36, 132) | 28.5 (+3, 123) |
| RB | 25.0 (-20, 52) | 31.2 (-17, 86) | 40.8 (-36, 132) | 28.5 (+3, 123) |
| RBS | 25.0 (-20, 52) | 31.2 (-17, 86) | 40.8 (-36, 132) | 28.5 (+3, 123) |
| RG | 18.5 (+7, 42) | 27.4 (+8, 70) | 40.4 (-14, 128) | 30.5 (+8, 101) |
| RGS | 18.5 (+7, 42) | 27.4 (+8, 70) | 40.4 (-14, 128) | 30.5 (+8, 101) |
| RGB | 18.5 (+7, 42) | 27.4 (+8, 70) | 40.4 (-14, 128) | 30.5 (+8, 101) |
| RGBS | 18.5 (+7, 42) | 27.4 (+8, 70) | 40.4 (-14, 128) | 30.5 (+8, 101) |
| joint@14 | 18.5 (+7, 42) | 27.4 (+8, 70) | 40.4 (-14, 128) | 30.5 (+8, 101) |
| joint@40 | 19.5 (+6, 45) | 27.5 (+4, 67) | 44.1 (-50, 134) | 29.5 (+5, 104) |
| deployed | 47.7 (-102, 145) | 42.6 (-74, 134) | 47.8 (-81, 142) | 32.1 (-3, 124) |

Cells: objective mm (median mm, p95 mm). Main effects (mm^2 of mean square per frame, 95 % block-bootstrap interval):

- warm-up (fit): R -78.1 [-123.5, -53.1]; G -324.4 [-355.0, -301.3]; B +0.0 [+0.0, +0.0]; S +0.0 [+0.0, +0.0]
- held out live_115658 40:120: R -34.6 [-57.3, -15.0]; G -253.7 [-318.5, -185.0]; B +0.0 [+0.0, +0.0]; S +0.0 [+0.0, +0.0]
- held out live_120408 20:90: R -379.3 [-438.3, -316.1]; G -101.8 [-128.7, -74.0]; B +0.0 [+0.0, +0.0]; S +0.0 [+0.0, +0.0]
- held out warmup_hardware_215321 10:100: R -56.6 [-99.8, -15.4]; G +33.7 [-22.7, +84.5]; B +0.0 [+0.0, +0.0]; S +0.0 [+0.0, +0.0]

Bending and shear one decade either side of the detected value on the held-out windows (objective mm):

- bend, held out live_115658 40:120: -1.00 31.06, +0.00 31.19, +1.00 31.47
- bend, held out live_120408 20:90: -1.00 43.83, +0.00 40.80, +1.00 38.99
- bend, held out warmup_hardware_215321 10:100: -1.00 30.46, +0.00 28.52, +1.00 30.78
- shear, held out live_115658 40:120: -1.00 30.58, +0.00 31.19, +1.00 34.24
- shear, held out live_120408 20:90: -1.00 38.75, +0.00 40.80, +1.00 45.21
- shear, held out warmup_hardware_215321 10:100: -1.00 26.92, +0.00 28.52, +1.00 31.88
- bend with grip points, held out live_115658 40:120: -1.00 27.32, +0.00 27.40, +1.00 27.63
- bend with grip points, held out live_120408 20:90: -1.00 40.38, +0.00 40.40, +1.00 44.75
- bend with grip points, held out warmup_hardware_215321 10:100: -1.00 30.43, +0.00 30.48, +1.00 31.12
- shear with grip points, held out live_115658 40:120: -1.00 27.42, +0.00 27.40, +1.00 29.09
- shear with grip points, held out live_120408 20:90: -1.00 43.08, +0.00 40.40, +1.00 42.69
- shear with grip points, held out warmup_hardware_215321 10:100: -1.00 25.50, +0.00 30.48, +1.00 30.97

Adopted (held-out evidence): R=True, G=True, B=False, S=False

R on = the coarser mesh instead of the finest; adopting R means the coarser mesh costs nothing measurable.
