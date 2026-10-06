#!/usr/bin/env python3
"""Summarize and compare benchmark runs made by bench.sh / matrix.sh.

usage:
  python3 tools/profiling/report.py [runs-dir]        per-run table, per-config summary, summary.csv
  python3 tools/profiling/report.py --timeline <run>  per-minute timeline of one run

env: WARMUP_S  seconds excluded at the start of each run (default 30)
     BIN_S     timeline bin width in seconds (default 60)
A stall is a frame interval above STALL_MS. Stalls in the warm-up are excluded too.
"""
import csv, math, os, re, statistics as st, sys, time

STALL_MS = 50.0
WARMUP_S = float(os.environ.get("WARMUP_S", 30))
BIN_S = float(os.environ.get("BIN_S", 60))
NAN = float("nan")


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))] if xs else NAN


def mean(xs):
    xs = list(xs)
    return st.mean(xs) if xs else NAN


def load_frames(path):
    frames = []
    for line in open(path, errors="ignore"):
        p = line.strip().split(",")
        # timestamp_ns, sdk_ns, conversion_ns, retries, objects[, enter_ns]
        if len(p) in (6, 7) and p[0] == "zed_frame" and p[1].isdigit():
            enter = int(p[6]) / 1e9 if len(p) == 7 else None
            frames.append((int(p[1]) / 1e9, int(p[2]) / 1e6, int(p[5]), int(p[3]) / 1e6, enter))
    return frames


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
    stalls = [(i, d) for i, d in enumerate(dt) if d > STALL_MS]
    stall_times = [ts[i + 1] for i, _ in stalls]
    gaps = [b - a for a, b in zip(stall_times, stall_times[1:])]
    # time the mission loop spent between camera calls (needs the enter_ns column)
    runner = ([(b[4] - a[4]) * 1000 - a[1] - a[3] for a, b in zip(steady, steady[1:])]
              if steady[0][4] is not None else [])
    tegra = [s for s in load_tegra(os.path.join(run, "tegra.log")) if ts[0] <= s["t"] <= ts[-1]]
    first = [s for s in tegra if s["t"] < ts[0] + 60]
    last = [s for s in tegra if s["t"] > ts[-1] - 60]
    return {
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

    print(f"== per run (first {WARMUP_S:.0f}s of each run excluded; stall = frame interval > {STALL_MS:.0f} ms)")
    table(results, ["run", "minutes", "fps", "p50_ms", "p99_ms", "p99.9_ms", "max_ms", "stalls_per_min",
                    "stall_every_s", "frames_lost", "stall_in_sdk_%", "stall_in_runner_%", "runner_max_ms", "gpu_mean_%", "gpu_p95_%", "cpu_mean_%",
                    "ram_max_mb", "ram_growth_mb", "power_mean_w", "temp_max_c", "objects"])

    configs = list(dict.fromkeys(r["config"] for r in results))
    keys = ["fps", "p99_ms", "p99.9_ms", "stalls_per_min", "frames_lost", "gpu_mean_%", "gpu_p95_%",
            "cpu_mean_%", "ram_max_mb", "power_mean_w", "temp_max_c"]
    summary = []
    for c in configs:
        group = [r for r in results if r["config"] == c]
        row = {"config": c, "n": len(group)}
        for k in keys:
            vals = [r[k] for r in group if not math.isnan(r[k])]
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
        w.writerows(results)
    print(f"\nwrote {out}")


def timeline(run):
    frames = load_frames(os.path.join(run, "zed.log"))
    if len(frames) < 3:
        sys.exit(f"no usable frames in {run}")
    tegra = load_tegra(os.path.join(run, "tegra.log"))
    t0 = frames[0][0]
    rows = []
    for b in range(int((frames[-1][0] - t0) // BIN_S) + 1):
        lo, hi = t0 + b * BIN_S, t0 + (b + 1) * BIN_S
        ts = [f[0] for f in frames if lo <= f[0] < hi]
        dt = [(y - x) * 1000 for x, y in zip(ts, ts[1:])]
        tg = [s for s in tegra if lo <= s["t"] < hi]
        if len(dt) < 2:
            continue
        rows.append({"start_min": f"{b * BIN_S / 60:.1f}", "fps": len(dt) / (ts[-1] - ts[0]),
                     "p99_ms": pct(dt, 99), "max_ms": max(dt), "stalls": sum(d > STALL_MS for d in dt),
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
