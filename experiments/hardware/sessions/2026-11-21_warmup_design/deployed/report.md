# Warm-up design study (2026-11-21 08:35)

Start corners from warmup_10_0.1_111137 (warm-up start). Truth = the cell twin (twin.yaml), depth sampled at 5 Hz, 128 points per frame. Detection limit 6.4 mm unique effect (model error 20 mm, threshold 1 mm).

| candidate | s | shear -1 | shear -0.5 | shear +0.5 | shear +1 | bend -1 | bend +1 | stretch -1 | CR sd shear | CR sd bend |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| deployed | 194 | 4.1 / 3.8 | 4.2 / 3.2 | 5.8 / 4.7 | 14.9 / 14.4 * | 1.2 / 1.1 | 5.4 / 5.2 | 12.2 / 11.8 * | 0.071 | 0.127 |

Cells: effect / unique (mm RMS of the depth residual); * = above the detection limit.

- deployed: geometry {"blue": 2.7, "silver": 3.0} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.93, "0-2": 5.66, "1-3": 5.49, "0-3": 3.45, "1-2": 3.96} cm; grip offsets 1 cm -> down_blue 5.3, down_silver 3.7, down_right 3.9, down_left 5.1, in_blue 4.6, in_silver 3.3, in_right 2.4, in_left 3.9 mm; CR sd offsets down_blue 0.10, down_silver 0.12, down_right 0.11, down_left 0.12, in_blue 0.12, in_silver 0.11, in_right 0.16, in_left 0.10 cm
