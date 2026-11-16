#!/usr/bin/env bash
# S4 closed-loop carries: 6 conditions x 3 repeats, interleaved by repeat so that slow drift in the
# machine's state does not line up with one condition. Fitted coefficients: the production (live
# input) S2 fit of the fine-B datasets, identical in all three repeats: bend 6.727, shear 0.420.
HERE=$(dirname "$(realpath "$0")")
FIT="6.727171322029717 0.4204482076268573"
ARGS=()
for r in 1 2 3; do
  ARGS+=("B0 B $r" "B1 B $r" "B2 B $r $FIT" "P0 B $r" "P B $r $FIT" "G0 B $r $FIT")
done
"$HERE/queue.sh" run_closed.sh "${ARGS[@]}"
