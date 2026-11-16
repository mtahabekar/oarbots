#!/usr/bin/env bash
# Run a list of experiments one after another (Isaac runs never share the GPU with each other).
#   queue.sh SCRIPT "ARGS1" "ARGS2" ...     e.g. queue.sh run_id.sh "fine B 1" "fine B 2"
# One summary line per run goes to stdout and to $SIM_EVAL_DATA/queue.log.
HERE=$(dirname "$(realpath "$0")")
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
SCRIPT=$1; shift
for args in "$@"; do
  started=$(date +%s)
  # shellcheck disable=SC2086
  line=$("$HERE/$SCRIPT" $args 2>&1 | tail -1)
  echo "$(date +%T) $SCRIPT $args -> $line ($(( $(date +%s)-started )) s)" | tee -a "$DATA/queue.log"
done
echo QUEUE_DONE
