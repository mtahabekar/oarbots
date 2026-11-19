# Stationary grip calibration, 18 November 2026

The operator specified that each robot holds the full 3 × 3 cm patch and that the far corner is
the grip origin. This matches the 0.03 m patch side in the twin. The clip transform still needs
the position of that corner relative to the end-effector frame.

At 19:05:40 EST the three Ethernet cameras recorded the two clips and the hanging stand-in.
Each saved nine RGB-D pairs. The [view check](images/grip_view_check.jpg) shows the tool-frame
origins projected into the images. These are reference pixels for locating the contacts.
Blue's contact side is at the left edge of Silver's wrist view, and the far corners cannot
be identified unambiguously in this capture. The wrist views have no registered depth near
the projected opposite tool origins. The overhead contains valid depth near the clips.
The [before](tf_before.json) and [after](tf_after.json) TF snapshots report identical tool
positions. The pose snapshots contain the surveyed world and current tool/camera transforms.

The three camera recordings are checksum-verified and saved at
`/media/taha/ExtermeLinux/claude-1000/lab/captures/2026-11-18_stationary_grip_calibration/grip_points_190540/`.
[Manifests](manifests/)

## Blue pointer measurement

At 19:10:36 EST the operator pointed to Blue's far corner. All three cameras saved seven
RGB-D pairs. The [annotated overhead frame](images/blue_pointer_reference.jpg) shows the point
used for the first estimate. Registered depth in the 5 × 5 neighbourhood ranged from 2.528
to 2.546 m, with a median of 2.536 m. This gives a provisional Blue tool-frame position of
`[26.8, 31.9, -6.4]` mm. The pointer tip is outside the opposite wrist view, so the estimate
still needs an independent sight line or a known-offset fixture. A subsequent
[TF snapshot](tf_pointer_check.json) matched the preceding tool and camera positions.
[Annotation](depth_annotations.json) · [Provisional result](depth_candidates.json)

The Blue pointer capture is under `blue_pointer_191036/` in the same archive.

[`grip_calibrate.py`](../../../../scripts/hw/grip_calibrate.py) uses pixel annotations of the
same point in calibrated RGB views to triangulate the position and report it in tool
coordinates. Its depth-only mode records provisional single-view measurements. A synthetic
check recovered a known translation and zero ray residual. Repeat
placements and cross-view agreement qualify the resulting grip transforms for S2.
