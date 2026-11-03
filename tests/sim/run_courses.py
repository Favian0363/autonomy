#!/usr/bin/env python3
"""Run the `waypoints` mission on many random courses, under one or more conditions, and summarize.

usage (from the repository root, after `zig build -Dstonefish`):
  python3 tests/sim/run_courses.py --seeds 20
  python3 tests/sim/run_courses.py --seeds 20 --conditions perfect jetson65
  python3 tests/sim/run_courses.py --seeds 20 --config my_gains.json   (compare controller gains)
  python3 tests/sim/run_courses.py --seeds 20 --conditions delay0 delay200 delay400   (delay sweep)

Each run is one course (one seed: the mission makes the course from it) under one condition, in
its own process, headless and faster than real time; several run at once (--jobs). Everything goes
into --out: one log per run, results.csv with one row per waypoint, and the summary that is also printed. The script exits
with 1 if any run failed, so it can be used in CI.

Conditions (what the mission sees):
  perfect   the simulator as is: 360 frames per second, no delay
  jetson65  30 frames per second, each 65 ms old: typical on the Jetson
  jetson115 30 frames per second, each 115 ms old: worst case measured with the detector
  delay<N>  30 frames per second, each N ms old (delay0: only the lower frame rate)
"""
import argparse
import csv
import os
import re
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

CONDITIONS = {
    "perfect": {},
    "jetson65": {"AUV_FAULT_RATE_HZ": "30", "AUV_FAULT_DELAY_MS": "65"},
    "jetson115": {"AUV_FAULT_RATE_HZ": "30", "AUV_FAULT_DELAY_MS": "115"},
}
RUNNER = "./zig-out/bin/src_3"
SIM = "./zig-out/lib/libauv_stonefish_sim.so"
FAULT = "./zig-out/lib/libauv_fault_layer.so"
CONFIG = "./auvs/stonefish_sim/auv.json"
# measurements per waypoint, as printed by the mission, and how to summarize them
NUMBERS = ["time_s", "final_dist_m", "overshoot_m", "max_roll_deg", "max_pitch_deg"]


def passed_through():
    """AUV_* settings from the caller's environment, which every run inherits (e.g.
    AUV_FAULT_FRESH_ATTITUDE or AUV_WAYPOINTS_TIMEOUT_S), except the ones this script sets."""
    ours = {"AUV_SIM_HEADLESS", "AUV_SIM_REALTIME", "AUV_WAYPOINTS_SEED", "AUV_WAYPOINTS_COUNT",
            "AUV_FAULT_TARGET", "AUV_FAULT_RATE_HZ", "AUV_FAULT_DELAY_MS"}
    return {k: v for k, v in sorted(os.environ.items()) if k.startswith("AUV_") and k not in ours}


def condition_env(name):
    """Fault layer settings for a condition name (empty: no fault layer)."""
    if name in CONDITIONS:
        return CONDITIONS[name]
    delay = re.fullmatch(r"delay(\d+)", name)
    if delay:
        return {"AUV_FAULT_RATE_HZ": "30", "AUV_FAULT_DELAY_MS": delay[1]}
    raise argparse.ArgumentTypeError(f"unknown condition {name!r}: use {', '.join(CONDITIONS)} or delay<ms>")


def mission_env(condition, seed, count):
    """Environment variables that pick the course and the condition; returns (env, plugin)."""
    fault = condition_env(condition)
    env = {"AUV_WAYPOINTS_SEED": str(seed), "AUV_WAYPOINTS_COUNT": str(count), **fault}
    if not fault:
        return env, SIM
    return {**env, "AUV_FAULT_TARGET": SIM}, FAULT


def run_one(condition, seed, count, out, timeout_s, config):
    """Runs one course under one condition; returns (status, waypoint rows)."""
    extra, plugin = mission_env(condition, seed, count)
    env = dict(os.environ, AUV_SIM_HEADLESS="1", AUV_SIM_REALTIME="0", **extra)
    log = out / "logs" / f"{condition}_seed{seed}.txt"
    try:
        with open(log, "w") as f:
            done = subprocess.run([RUNNER, plugin, "waypoints", config], env=env, stdout=f,
                                  stderr=subprocess.STDOUT, timeout=timeout_s)
        code = done.returncode
    except subprocess.TimeoutExpired:
        code = None
    text = log.read_text(errors="replace")
    rows = []
    for line in re.findall(r"waypoint,seed=[^\n\x1b]*", text):
        fields = dict(part.split("=", 1) for part in line.split(",goal=")[0].split(",")[1:])
        fields["goal"] = line.split(",goal=")[1] if ",goal=" in line else ""
        rows.append({"condition": condition, **fields})
    if code is None:
        status = "timeout"
    elif code == 0:
        status = "passed"
    elif "SimulationDiverged" in text:
        status = "diverged"
    elif "WaypointsMissed" in text:
        status = "missed"
    else:
        status = f"crashed (exit {code})"
    return status, rows


def rerun_command(condition, seed, count, config):
    """Shell command that replays one run with the window open."""
    env, plugin = mission_env(condition, seed, count)
    env = {**passed_through(), **env}
    return " ".join(f"{k}={v}" for k, v in env.items()) + f" {RUNNER} {plugin} waypoints {config}"


def summarize(condition, statuses, rows):
    runs = len(statuses)
    failed = {seed: s for seed, s in statuses.items() if s != "passed"}
    reached = sum(r["reached"] == "true" for r in rows)
    held = sum(r["held"] == "true" for r in rows)
    lines = [f"== {condition}: {runs - len(failed)}/{runs} runs passed, "
             f"waypoints reached {reached}/{len(rows)}, held {held}/{len(rows)}"]
    if not failed and runs > 3:
        # with no failures in n runs, a failure rate above about 3/n would likely have shown one
        lines.append(f"   no failures: the failure rate is probably below {300 / runs:.0f}% (95% confidence)")
    for name in NUMBERS:
        values = [float(r[name]) for r in rows if r.get(name)]
        if values:
            lines.append(f"   {name:15} mean {statistics.mean(values):7.3f}   max {max(values):7.3f}")
    for seed, status in sorted(failed.items()):
        lines.append(f"   seed {seed}: {status}")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, default=10, help="run seeds 1..N (default 10)")
    parser.add_argument("--count", type=int, default=8, help="waypoints per course (default 8)")
    parser.add_argument("--conditions", nargs="+", default=list(CONDITIONS), metavar="CONDITION",
                        type=lambda name: condition_env(name) is not None and name,
                        help="perfect, jetson65, jetson115 or delay<ms> (default: the first three)")
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("--timeout", type=float, default=900, help="seconds of real time per run")
    parser.add_argument("--out", type=Path, default=Path("sim_results"))
    parser.add_argument("--config", default=CONFIG, help=f"controller config (default {CONFIG})")
    args = parser.parse_args()

    for path in [RUNNER, SIM, FAULT, args.config, "auvs/stonefish_sim/data"]:
        if not Path(path).exists():
            sys.exit(f"missing {path}: run from the repository root after `zig build -Dstonefish`")
    if args.out.exists():
        sys.exit(f"{args.out} already exists; choose another --out so results are never mixed")
    (args.out / "logs").mkdir(parents=True)

    jobs = [(c, s) for c in args.conditions for s in range(1, args.seeds + 1)]
    header = [f"config {args.config}"] + [f"env {k}={v}" for k, v in passed_through().items()]
    print("\n".join(header))
    print(f"{len(jobs)} runs ({len(args.conditions)} conditions x {args.seeds} seeds), {args.jobs} at a time")
    with ThreadPoolExecutor(args.jobs) as pool:
        results = list(pool.map(lambda job: run_one(*job, args.count, args.out, args.timeout, args.config), jobs))

    all_rows, report = [], list(header)
    for condition in args.conditions:
        statuses, rows = {}, []
        for (c, seed), (status, run_rows) in zip(jobs, results):
            if c == condition:
                statuses[seed] = status
                rows += run_rows
        all_rows += rows
        report += summarize(condition, statuses, rows)
    failed = [(c, s) for (c, s), (status, _) in zip(jobs, results) if status != "passed"]
    if failed:
        c, s = failed[0]
        report.append(f"log of the first failure: {args.out}/logs/{c}_seed{s}.txt; to watch it, run:")
        report.append("  " + rerun_command(c, s, args.count, args.config))

    with open(args.out / "results.csv", "w", newline="") as f:
        names = ["condition", "seed", "index", "reached", "held"] + NUMBERS + ["min_dist_m", "goal"]
        writer = csv.DictWriter(f, fieldnames=names, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_rows)
    (args.out / "summary.txt").write_text("\n".join(report) + "\n")
    print("\n".join(report))
    print(f"wrote {args.out}/results.csv and {args.out}/summary.txt")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
