# Warm-up design study (2026-11-21 08:18)

Start corners from warmup_10_0.1_111137 (warm-up start). Truth = the cell twin (twin.yaml), depth sampled at 5 Hz, 128 points per frame. Detection limit 6.4 mm unique effect (model error 20 mm, threshold 1 mm).

| candidate | s | shear -1 | shear -0.5 | shear +0.5 | shear +1 | bend -1 | bend +1 | stretch -1 | CR sd shear | CR sd bend |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| hands up/down | 60 | 4.9 / 3.4 | 4.9 / 2.9 | 4.7 / 3.9 | 18.0 / 15.6 * | 0.6 / 0.4 | 5.0 / 3.3 | 11.1 / 10.1 * | 0.146 | 0.331 |
| hands toward | 38 | 3.9 / 3.3 | 2.8 / 2.3 | 6.1 / 4.8 | 21.9 / 17.6 * | 1.1 / 1.0 | 2.5 / 2.3 | 11.3 / 10.4 * | 0.168 | 0.469 |
| hands together | 54 | 3.6 / 3.1 | 2.5 / 2.2 | 6.3 / 4.9 | 20.0 / 16.7 * | 1.7 / 1.6 | 3.1 / 2.6 | 11.4 / 10.7 * | 0.144 | 0.353 |
| hands side | 63 | 4.3 / 3.6 | 2.9 / 2.6 | 5.1 / 4.6 | 17.8 / 15.5 * | 1.4 / 1.2 | 2.8 / 2.7 | 12.8 / 12.0 * | 0.128 | 0.342 |
| hands all | 113 | 5.2 / 4.5 | 4.4 / 3.7 | 4.6 / 3.9 | 15.5 / 14.0 * | 0.8 / 0.8 | 5.3 / 4.5 | 11.9 / 11.1 * | 0.097 | 0.160 |
| robots all + hands all | 185 | 3.2 / 2.9 | 4.0 / 2.7 | 5.0 / 4.2 | 13.8 / 13.3 * | 0.9 / 0.8 | 4.9 / 4.6 | 10.6 / 10.3 * | 0.085 | 0.152 |

Cells: effect / unique (mm RMS of the depth residual); * = above the detection limit.

- hands up/down: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 0.0, "0-2": 2.39, "1-3": 2.23, "0-3": 1.36, "1-2": 1.64} cm; grip offsets 1 cm -> down_blue 4.2, down_silver 3.9, down_right 3.8, down_left 5.2, in_blue 4.3, in_silver 4.3, in_right 5.1, in_left 5.3 mm; CR sd offsets down_blue 0.26, down_silver 0.25, down_right 0.26, down_left 0.27, in_blue 0.32, in_silver 0.20, in_right 0.24, in_left 0.17 cm
- hands toward: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 0.0, "1-3": 0.0, "0-3": 0.0, "1-2": 0.0} cm; grip offsets 1 cm -> down_blue 3.9, down_silver 3.8, down_right 4.3, down_left 3.9, in_blue 2.8, in_silver 4.3, in_right 2.5, in_left 6.0 mm; CR sd offsets down_blue 0.34, down_silver 0.34, down_right 0.30, down_left 0.35, in_blue 0.40, in_silver 0.25, in_right 0.38, in_left 0.20 cm
- hands together: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 0.0, "1-3": 0.0, "0-3": 0.0, "1-2": 0.0} cm; grip offsets 1 cm -> down_blue 4.2, down_silver 4.2, down_right 4.2, down_left 4.3, in_blue 2.5, in_silver 3.6, in_right 2.2, in_left 6.2 mm; CR sd offsets down_blue 0.29, down_silver 0.21, down_right 0.22, down_left 0.29, in_blue 0.33, in_silver 0.21, in_right 0.34, in_left 0.15 cm
- hands side: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 0.0, "1-3": 0.0, "0-3": 0.0, "1-2": 0.0} cm; grip offsets 1 cm -> down_blue 3.7, down_silver 3.8, down_right 4.1, down_left 4.1, in_blue 2.7, in_silver 4.8, in_right 2.4, in_left 4.9 mm; CR sd offsets down_blue 0.24, down_silver 0.26, down_right 0.23, down_left 0.22, in_blue 0.30, in_silver 0.18, in_right 0.30, in_left 0.17 cm
- hands all: geometry {"blue": 7.8, "silver": 7.9} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 2.39, "1-3": 2.23, "0-3": 1.36, "1-2": 1.64} cm; grip offsets 1 cm -> down_blue 4.1, down_silver 4.5, down_right 4.5, down_left 4.6, in_blue 3.7, in_silver 3.7, in_right 5.2, in_left 4.7 mm; CR sd offsets down_blue 0.18, down_silver 0.13, down_right 0.13, down_left 0.18, in_blue 0.19, in_silver 0.14, in_right 0.12, in_left 0.12 cm
- robots all + hands all: geometry {"blue": 2.7, "silver": 3.0} cm to the box faces, pair growth {"0-1": 0.0, "2-3": 1.16, "0-2": 2.39, "1-3": 2.23, "0-3": 1.36, "1-2": 1.64} cm; grip offsets 1 cm -> down_blue 5.3, down_silver 3.7, down_right 3.9, down_left 4.8, in_blue 4.0, in_silver 3.3, in_right 2.4, in_left 3.9 mm; CR sd offsets down_blue 0.12, down_silver 0.13, down_right 0.13, down_left 0.14, in_blue 0.14, in_silver 0.12, in_right 0.18, in_left 0.10 cm
