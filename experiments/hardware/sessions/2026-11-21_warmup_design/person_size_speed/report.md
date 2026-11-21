# Warm-up design study (2026-11-21 08:25)

Start corners from warmup_10_0.1_111137 (warm-up start). Truth = the cell twin (twin.yaml), depth sampled at 5 Hz, 128 points per frame. Detection limit 6.4 mm unique effect (model error 20 mm, threshold 1 mm).

| candidate | s | shear -1 | shear -0.5 | shear +0.5 | shear +1 | bend -1 | bend +1 | stretch -1 | CR sd shear | CR sd bend |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| hands large | 123 | 6.5 / 5.2 | 4.2 / 3.6 | 6.8 / 5.0 | 17.5 / 15.4 * | 4.4 / 2.4 | 7.3 / 5.0 | 13.8 / 13.0 * | 0.077 | 0.119 |
| hands fast | 32 | 6.2 / 4.2 | 4.6 / 3.0 | 6.5 / 5.5 | 24.1 / 19.4 * | 1.2 / 1.2 | 3.6 / 3.1 | 15.5 / 13.5 * | 0.145 | 0.488 |

Cells: effect / unique (mm RMS of the depth residual); * = above the detection limit.

- hands large: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.93, "0-2": 5.66, "1-3": 5.49, "0-3": 3.45, "1-2": 3.96} cm; grip offsets 1 cm -> down_blue 6.1, down_silver 5.5, down_right 4.4, down_left 5.8, in_blue 5.6, in_silver 5.1, in_right 4.4, in_left 5.9 mm; CR sd offsets down_blue 0.14, down_silver 0.10, down_right 0.13, down_left 0.12, in_blue 0.14, in_silver 0.13, in_right 0.12, in_left 0.10 cm
- hands fast: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 2.39, "1-3": 2.23, "0-3": 1.36, "1-2": 1.64} cm; grip offsets 1 cm -> down_blue 4.6, down_silver 3.4, down_right 3.9, down_left 5.0, in_blue 4.0, in_silver 5.5, in_right 2.6, in_left 5.8 mm; CR sd offsets down_blue 0.32, down_silver 0.35, down_right 0.31, down_left 0.35, in_blue 0.38, in_silver 0.24, in_right 0.40, in_left 0.23 cm
