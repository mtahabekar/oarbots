# S2 repeat at 20:27 EST, 18 November 2026

The operator reported that the overlay worked much better. The new estimate contains 1,209
states, including 566 camera analyses. The 38 s window through the last fused glove sample
ran at 4.84 analyses/s, with 197 ms median and 441 ms p95 capture-to-recorder age. The largest
successive camera-analysis displacement was 52.6 cm at the right-hand corner, compared
with 65.7 cm in the [earlier repeat](../repeat_20261118_195112/README.md). The runs have no
matched motion cues, so this comparison describes the saved states and leaves the
reason for the operator-observed improvement open.

Both wrists supplied observations. Overhead perception again rejected the input timestamps and
published zero observations. All three camera hosts still reported RPI campus NTP as their
active clock source when checked after the run.
[Camera logs](cameras/) · [Estimate](estimate.npz)

The recorder reused the earlier local output path. This directory preserves the new outputs
separately, and the earlier outputs were recovered from the verified archive.
