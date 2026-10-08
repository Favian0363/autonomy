#!/usr/bin/env bash
# Repeated, interleaved comparison (A B C, A B C, ...) so slow drift such as heat
# affects every configuration equally. Each config is "name|VAR=value VAR=value".
# usage: tools/profiling/matrix.sh ["name|settings" ...]
#   default configs: baseline, area_memory_off, depth_light, plus checker_control when
#   CHECKER points at Ian's camera_check (same plugin and settings as baseline)
# env: REPS (default 3), DURATION (default 300), COOLDOWN (default 60), RUNS_DIR
set -uo pipefail

here=$(cd "$(dirname "$0")" && pwd)
export RUNS_DIR=${RUNS_DIR:-$HOME/profiling/runs/matrix_$(date +%Y%m%d_%H%M)}
export DURATION=${DURATION:-300} COOLDOWN=${COOLDOWN:-60}
if [ $# -gt 0 ]; then
  configs=("$@")
else
  configs=("baseline|" "area_memory_off|AUV_ZED_AREA_MEMORY=0" "depth_light|AUV_ZED_DEPTH=NEURAL_LIGHT")
  [ -n "${CHECKER:-}" ] && configs+=("checker_control|CHECKER=$CHECKER")
fi

echo "[matrix] ${REPS:-3} reps x ${#configs[@]} configs x ${DURATION}s -> $RUNS_DIR"
for rep in $(seq 1 "${REPS:-3}"); do
  for config in "${configs[@]}"; do
    name=${config%%|*}
    read -r -a settings <<< "${config#*|}"
    "$here/bench.sh" "r${rep}_${name}" "${settings[@]}" || echo "[matrix] r${rep}_${name} failed, continuing"
  done
done
python3 "$here/report.py" "$RUNS_DIR"
