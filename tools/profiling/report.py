#!/usr/bin/env python3
"""Summarize and compare benchmark runs made by bench.sh / matrix.sh.

usage:
  python3 tools/profiling/report.py [runs-dir]        per-run table, per-config summary, summary.csv
  python3 tools/profiling/report.py --timeline <run>  per-minute timeline of one run

env: WARMUP_S     seconds excluded at the start of each run (default 30)
     BIN_S        timeline bin width in seconds (default 60)
     DEADLINE_MS  mission loop budget; longer loop periods count as deadline misses (default 100)
     MIN_RATE_HZ  minimum effective system rate the KPI must meet at p99 (default 10)
A stall is a frame interval above STALL_FACTOR x the run's median interval (50 ms at 30 FPS,
25 ms at 60 FPS), so one missed frame counts at any frame rate. Warm-up stalls are excluded.
Columns are read by name from the zed_frame header, so logs from older driver versions
still work; KPIs that need newer columns are reported as "-".
"""
import csv, math, os, re, statistics as st, sys, time

STALL_FACTOR = 1.5
WARMUP_S = float(os.environ.get("WARMUP_S", 30))
BIN_S = float(os.environ.get("BIN_S", 60))
DEADLINE_MS = float(os.environ.get("DEADLINE_MS", 100))
MIN_RATE_HZ = float(os.environ.get("MIN_RATE_HZ", 10))
OLD_HEADER = ["timestamp_ns", "sdk_ns", "conversion_ns", "retries", "objects", "enter_ns"]
NAN = float("nan")


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))] if xs else NAN


def mean(xs):
    xs = list(xs)
    return st.mean(xs) if xs else NAN


def load_log(path):
    """Rows of a driver metrics log as dicts keyed by header name, plus clock info and error events."""
    header, rows, clock, errors = OLD_HEADER, [], None, {}
    for line in open(path, errors="ignore"):
        p = line.strip().split(",")
        if p[0] == "zed_clock" and len(p) == 4:
            clock = (p[1], int(p[2]), int(p[3]))
        elif p[0] == "zed_error" and len(p) >= 4:
            key = f"{p[2]}:{','.join(p[3:])}"
            errors[key] = errors.get(key, 0) + 1
        elif p[0] == "zed_frame" and len(p) > 1 and not p[1].isdigit():
            header = p[1:]
        elif p[0] == "zed_frame" and len(p) > 1 and p[1].isdigit():
            values = p[1:]
            if len(values) <= len(header):
                rows.append(dict(zip(header, (int(v) for v in values))))
    return rows, clock, errors


def sdk_to_wall_s(ns, clock):
    """SDK timestamps are wall clock unless the driver logged a MONOTONIC zed_clock reference."""
    if clock and clock[0] == "MONOTONIC":
        return (ns - clock[1] + clock[2]) / 1e9
    return ns / 1e9


def sdk_to_mono_ns(ns, clock):
    """SDK timestamp on the monotonic clock used by enter_ns/exit_ns; None if it cannot be known."""
    if not clock:
        return None
    return ns if clock[0] == "MONOTONIC" else ns - clock[2] + clock[1]


def load_frames(path):
    rows, clock, _ = load_log(path)
    return [(sdk_to_wall_s(r["timestamp_ns"], clock), r["sdk_ns"] / 1e6, r.get("objects", 0),
             r.get("conversion_ns", 0) / 1e6, r["enter_ns"] / 1e9 if "enter_ns" in r else None)
            for r in rows if "timestamp_ns" in r and "sdk_ns" in r]


def kpi_stats(run, steady_start):
    """Cesar's KPIs: effective rate, data age, loop determinism, errors. Needs the newer driver columns."""
    rows, clock, errors = load_log(os.path.join(run, "zed.log"))
    out = {"errors": errors}
    if not rows or "exit_ns" not in rows[0]:
        return out
    rows = [r for r in rows if sdk_to_wall_s(r["timestamp_ns"], clock) >= steady_start] or rows
    span_s = (rows[-1]["exit_ns"] - rows[0]["exit_ns"]) / 1e9
    period = [(b["enter_ns"] - a["enter_ns"]) / 1e6 for a, b in zip(rows, rows[1:])]
    out.update({
        "loop_p50_ms": pct(period, 50), "loop_p99_ms": pct(period, 99), "loop_max_ms": max(period, default=NAN),
        "deadline_miss": sum(p > DEADLINE_MS for p in period),
        "grab_fail": sum(r["grab_fail"] for r in rows), "track_bad": sum(r["track_bad"] for r in rows),
    })
    age = lambda key: [(r["exit_ns"] - sdk_to_mono_ns(r[key], clock)) / 1e6 for r in rows
                       if r[key] and sdk_to_mono_ns(r[key], clock) is not None]
    img_age, pose_age = age("timestamp_ns"), age("pose_ns")
    out.update({"img_age_p50_ms": pct(img_age, 50), "img_age_p99_ms": pct(img_age, 99),
                "pose_age_p99_ms": pct(pose_age, 99),
                "pose_stale_%": 100 * mean(r["pose_ns"] != r["timestamp_ns"] for r in rows)})
    detection = [r for r in rows if r["objects_ns"]]
    if detection:
        new = [r["objects_ns"] for r in detection if r["objects_new"]]
        gaps = [(b - a) / 1e6 for a, b in zip(new, new[1:])]
        det_age = age("objects_ns")
        out.update({"det_rate_hz": len(new) / span_s if span_s else NAN,
                    "det_gap_p99_ms": pct(gaps, 99), "det_gap_max_ms": max(gaps, default=NAN),
                    "det_age_p50_ms": pct(det_age, 50), "det_age_p99_ms": pct(det_age, 99)})
        slowest_gap = out["det_gap_p99_ms"]
    else:
        slowest_gap = out["loop_p99_ms"]
    # KPI 1: rate of the slowest output the mission depends on, judged at p99 rather than on average
    out["eff_rate_p99_hz"] = 1000 / slowest_gap if slowest_gap and not math.isnan(slowest_gap) else NAN
    out["kpi_min_rate"] = ("PASS" if out["eff_rate_p99_hz"] >= MIN_RATE_HZ else "FAIL") \
        if not math.isnan(out["eff_rate_p99_hz"]) else "-"
    return out


def load_tegra(path):
    samples = []
    if not os.path.exists(path):
        return samples
    for line in open(path, errors="ignore"):
        m = re.match(r"(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d)", line)
        if not m:
            continue
        s = {"t": time.mktime(time.strptime(m[1], "%m-%d-%Y %H:%M:%S"))}
        if g := re.search(r"GR3D_FREQ (\d+)%", line):
            s["gpu"] = int(g[1])
        if c := re.search(r"CPU \[([^\]]*)\]", line):
            loads = [int(x) for x in re.findall(r"(\d+)%@", c[1])]
            if loads:
                s["cpu"], s["core"] = st.mean(loads), max(loads)
        if r := re.search(r"RAM (\d+)/", line):
            s["ram"] = int(r[1])
        if p := re.search(r"VDD_IN (\d+)mW", line):
            s["power"] = int(p[1]) / 1000
        temps = [float(x) for x in re.findall(r"@(-?[\d.]+)C", line)]
        if temps:
            s["temp"] = max(temps)
        samples.append(s)
    return samples


def col(samples, key):
    return [s[key] for s in samples if key in s]


def analyze(run):
    frames = load_frames(os.path.join(run, "zed.log"))
    if len(frames) < 3:
        return None
    t0 = frames[0][0]
    steady = [f for f in frames if f[0] - t0 >= WARMUP_S] or frames
    ts = [f[0] for f in steady]
    dt = [(b - a) * 1000 for a, b in zip(ts, ts[1:])]
    if not dt:
        return None
    span = ts[-1] - ts[0]
    nominal = pct(dt, 50)
    stalls = [(i, d) for i, d in enumerate(dt) if d > STALL_FACTOR * nominal]
    stall_times = [ts[i + 1] for i, _ in stalls]
    gaps = [b - a for a, b in zip(stall_times, stall_times[1:])]
    # time the mission loop spent between camera calls (needs the enter_ns column)
    runner = ([(b[4] - a[4]) * 1000 - a[1] - a[3] for a, b in zip(steady, steady[1:])]
              if steady[0][4] is not None else [])
    tegra = [s for s in load_tegra(os.path.join(run, "tegra.log")) if ts[0] <= s["t"] <= ts[-1]]
    first = [s for s in tegra if s["t"] < ts[0] + 60]
    last = [s for s in tegra if s["t"] > ts[-1] - 60]
    kpi = kpi_stats(run, ts[0])
    return {
        **kpi,
        "frames": len(steady),
        "minutes": span / 60,
        "fps": len(dt) / span,
        "p50_ms": nominal,
        "p99_ms": pct(dt, 99),
        "p99.9_ms": pct(dt, 99.9),
        "max_ms": max(dt),
        "stalls": len(stalls),
        "stalls_per_min": len(stalls) / (span / 60),
        "stall_every_s": st.median(gaps) if gaps else NAN,
        "stall_median_ms": st.median(d for _, d in stalls) if stalls else NAN,
        "frames_lost": sum(max(0, round(d / nominal) - 1) for _, d in stalls),
        "stall_in_sdk_%": 100 * mean(min(1, steady[i + 1][1] / d) for i, d in stalls) if stalls else NAN,
        "stall_in_runner_%": 100 * mean(min(1, max(0, runner[i]) / d) for i, d in stalls) if stalls and runner else NAN,
        "sdk_p99_ms": pct([f[1] for f in steady], 99),
        "runner_p99_ms": pct(runner, 99),
        "runner_max_ms": max(runner, default=NAN),
        "objects": mean(f[2] for f in steady),
        "tegra_samples": len(tegra),
        "gpu_mean_%": mean(col(tegra, "gpu")),
        "gpu_p95_%": pct(col(tegra, "gpu"), 95),
        "cpu_mean_%": mean(col(tegra, "cpu")),
        "core_p95_%": pct(col(tegra, "core"), 95),
        "ram_max_mb": max(col(tegra, "ram"), default=NAN),
        "ram_growth_mb": mean(col(last, "ram")) - mean(col(first, "ram")),
        "power_mean_w": mean(col(tegra, "power")),
        "temp_max_c": max(col(tegra, "temp"), default=NAN),
        "temp_rise_c": mean(col(last, "temp")) - mean(col(first, "temp")),
    }


def run_info(run):
    path = os.path.join(run, "run_info.txt")
    info = dict(l.split(": ", 1) for l in open(path).read().splitlines() if ": " in l) if os.path.exists(path) else {}
    return {k: v.strip() for k, v in info.items()}


def fmt(v):
    if isinstance(v, float):
        return "-" if math.isnan(v) else f"{v:.1f}" if abs(v) >= 10 else f"{v:.2f}"
    return str(v)


def table(rows, cols):
    widths = [max(len(c), *(len(fmt(r.get(c, ""))) for r in rows)) for c in cols]
    print("  ".join(c.ljust(w) for c, w in zip(cols, widths)))
    for r in rows:
        print("  ".join(fmt(r.get(c, "")).ljust(w) for c, w in zip(cols, widths)))


def compare(root):
    if not os.path.isdir(root):
        sys.exit(f"no runs directory {root} (did the benchmark start? check the bench.sh output)")
    results = []
    for name in sorted(os.listdir(root)):
        run = os.path.join(root, name)
        if not os.path.isfile(os.path.join(run, "zed.log")):
            continue
        stats = analyze(run)
        info = run_info(run)
        if stats is None:
            print(f"!! {name}: no usable frames (see {run}/zed.log)")
            continue
        results.append({"run": name, "config": re.sub(r"^r\d+_", "", name),
                        "settings": info.get("settings") or "defaults", **stats})
    if not results:
        sys.exit(f"no runs found in {root}")

    print(f"== per run (first {WARMUP_S:.0f}s of each run excluded; stall = frame interval > {STALL_FACTOR}x the median)")
    table(results, ["run", "minutes", "fps", "p50_ms", "p99_ms", "p99.9_ms", "max_ms", "stalls_per_min",
                    "stall_every_s", "frames_lost", "stall_in_sdk_%", "stall_in_runner_%", "runner_max_ms", "gpu_mean_%", "gpu_p95_%", "cpu_mean_%",
                    "ram_max_mb", "ram_growth_mb", "power_mean_w", "temp_max_c", "objects"])

    print(f"\n== KPIs (deadline {DEADLINE_MS:.0f} ms, minimum effective rate {MIN_RATE_HZ:.0f} Hz at p99; "
          f"ages are capture -> handed to the mission)")
    table(results, ["run", "eff_rate_p99_hz", "kpi_min_rate", "loop_p50_ms", "loop_p99_ms", "loop_max_ms",
                    "deadline_miss", "img_age_p50_ms", "img_age_p99_ms", "pose_age_p99_ms", "pose_stale_%",
                    "det_rate_hz", "det_gap_p99_ms", "det_age_p99_ms", "grab_fail", "track_bad"])
    for r in results:
        if r.get("errors"):
            print(f"  {r['run']} error events: " + ", ".join(f"{k} x{v}" for k, v in sorted(r["errors"].items())))

    configs = list(dict.fromkeys(r["config"] for r in results))
    keys = ["fps", "p99_ms", "p99.9_ms", "stalls_per_min", "frames_lost", "eff_rate_p99_hz", "loop_p99_ms",
            "deadline_miss", "img_age_p99_ms", "det_rate_hz", "det_age_p99_ms", "gpu_mean_%", "gpu_p95_%",
            "cpu_mean_%", "ram_max_mb", "power_mean_w", "temp_max_c"]
    summary = []
    for c in configs:
        group = [r for r in results if r["config"] == c]
        row = {"config": c, "n": len(group)}
        for k in keys:
            vals = [r[k] for r in group if isinstance(r.get(k), (int, float)) and not math.isnan(r[k])]
            row[k] = (f"{fmt(st.mean(vals))}±{fmt(st.stdev(vals))}" if len(vals) > 1
                      else fmt(vals[0]) if vals else "-")
        summary.append(row)
    print("\n== per config (mean ± standard deviation across repeats)")
    table(summary, ["config", "n"] + keys)

    print("\n== settings and environment")
    seen = set()
    for r in results:
        if r["config"] not in seen:
            seen.add(r["config"])
            print(f"  {r['config']}: {r['settings']}")
    info = run_info(os.path.join(root, results[0]["run"]))
    for k in ("commit", "l4t", "zed_sdk", "power_mode"):
        print(f"  {k}: {info.get(k, 'unknown')}")

    out = os.path.join(root, "summary.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in results for k in r)))
        w.writeheader()
        w.writerows({k: "; ".join(f"{e} x{n}" for e, n in v.items()) if isinstance(v, dict) else v
                     for k, v in r.items()} for r in results)
    print(f"\nwrote {out}")


def timeline(run):
    frames = load_frames(os.path.join(run, "zed.log"))
    if len(frames) < 3:
        sys.exit(f"no usable frames in {run}")
    tegra = load_tegra(os.path.join(run, "tegra.log"))
    t0 = frames[0][0]
    all_ts = [f[0] for f in frames]
    threshold = STALL_FACTOR * pct([(b - a) * 1000 for a, b in zip(all_ts, all_ts[1:])], 50)
    rows = []
    for b in range(int((frames[-1][0] - t0) // BIN_S) + 1):
        lo, hi = t0 + b * BIN_S, t0 + (b + 1) * BIN_S
        ts = [f[0] for f in frames if lo <= f[0] < hi]
        dt = [(y - x) * 1000 for x, y in zip(ts, ts[1:])]
        tg = [s for s in tegra if lo <= s["t"] < hi]
        if len(dt) < 2:
            continue
        rows.append({"start_min": f"{b * BIN_S / 60:.1f}", "fps": len(dt) / (ts[-1] - ts[0]),
                     "p99_ms": pct(dt, 99), "max_ms": max(dt), "stalls": sum(d > threshold for d in dt),
                     "gpu_mean_%": mean(col(tg, "gpu")), "cpu_mean_%": mean(col(tg, "cpu")),
                     "ram_mb": mean(col(tg, "ram")), "power_w": mean(col(tg, "power")),
                     "temp_max_c": max(col(tg, "temp"), default=NAN)})
    print(f"== timeline of {run} ({BIN_S:.0f}s bins, warm-up included)")
    table(rows, list(rows[0].keys()))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--timeline":
        timeline(os.path.expanduser(sys.argv[2]))
    else:
        compare(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/profiling/runs"))
