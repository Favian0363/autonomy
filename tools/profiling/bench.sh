#!/usr/bin/env bash
# Runs the ZED pipeline for a fixed time while logging tegrastats, one folder per run.
# usage: tools/profiling/bench.sh <run-name> [VAR=value ...]
#   e.g. tools/profiling/bench.sh area_off AUV_ZED_AREA_MEMORY=0
# env: DURATION (seconds, default 300), COOLDOWN (seconds after the run, default 60),
#      RUNS_DIR (default ~/profiling/runs)
set -euo pipefail

name=${1:?usage: bench.sh <run-name> [VAR=value ...]}
shift
repo=$(cd "$(dirname "$0")/../.." && pwd)
out=${RUNS_DIR:-$HOME/profiling/runs}/$name
duration=${DURATION:-300}
[ -e "$out" ] && { echo "refusing to overwrite $out" >&2; exit 1; }

# the ZED can only be opened by one program, and other ZED tools would skew the measurements
if others=$(pgrep -l '^(ZED_|ZEDfu|src_3$)'); then
  echo "[bench] another ZED program is running, not starting $name:" >&2
  echo "$others" >&2
  exit 1
fi
if command -v lsusb >/dev/null && ! lsusb | grep -qi '2b03:'; then
  echo "[bench] no ZED camera on USB (lsusb shows no 2b03 device), not starting $name" >&2
  exit 1
fi
mkdir -p "$out"

sdk_version=$(grep -h -E 'define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/*.hpp 2>/dev/null \
  | awk '{print $3}' | paste -sd. - || true)
{
  echo "date: $(date -Iseconds)"
  echo "host: $(hostname)"
  echo "commit: $(git -C "$repo" rev-parse --short HEAD 2>/dev/null || echo unknown)$(git -C "$repo" diff --quiet 2>/dev/null || echo ' (modified)')"
  echo "l4t: $(head -1 /etc/nv_tegra_release 2>/dev/null || echo unknown)"
  echo "zed_sdk: ${sdk_version:-unknown}"
  echo "power_mode: $(nvpmodel -q 2>/dev/null | head -1 || echo unknown)"
  echo "duration_s: $duration"
  echo "settings: $*"
} > "$out/run_info.txt"

tegrastats --interval 500 --logfile "$out/tegra.log" &
tegra_pid=$!
trap 'kill $tegra_pid 2>/dev/null || true' EXIT

echo "[bench] $(date +%H:%M:%S) $name: running ${duration}s with: ${*:-defaults}"
cd "$repo"
env AUV_ZED_METRICS=1 "$@" timeout -s INT -k 10 "$duration" \
  ./zig-out/bin/src_3 ./zig-out/lib/libauv_proteus_hwd.so prequalify auvs/proteus_hwd/auv.json \
  > "$out/stdout.log" 2> "$out/zed.log" || true

kill $tegra_pid 2>/dev/null || true
frames=$(grep -c '^zed_frame,[0-9]' "$out/zed.log" || true)
echo "[bench] $(date +%H:%M:%S) $name: $frames frames recorded -> $out"
if [ "$frames" -eq 0 ]; then
  echo "[bench] no frames; first errors:" >&2
  grep -h -v 'cudaErrorCudartUnloading' "$out/zed.log" "$out/stdout.log" | grep -i -m 5 -E 'error|fail' >&2 || true
fi
sleep "${COOLDOWN:-60}"
