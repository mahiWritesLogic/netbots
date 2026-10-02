"""Run an experiment config: every variant x seed, plus an ideal-network reference per robot count.

    python experiments/run.py experiments/configs/baseline.json [--seeds 1 2] [--duration 20]

Writes results/<name>/{summary.csv, meta.json, *.png} and one directory per run.
"""
import argparse
import csv
import json
import os
import platform
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
import mujoco
import numpy

from analysis import metrics, plots
from integration import cosim

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def variants(spec):
    if "sweep" in spec:
        p = spec["sweep"]["param"]
        return [{"label": f"{p}={v}", p: v} for v in spec["sweep"]["values"]]
    return spec.get("variants", [{"label": "base"}])


def versions():
    ns3 = os.path.basename(os.path.realpath(cosim.NS3_DIR))
    try:
        ns3_bin = cosim.ns3_binary()
    except FileNotFoundError:
        ns3_bin = None
    return dict(ns3=ns3, ns3_binary=ns3_bin, mujoco=mujoco.__version__, numpy=numpy.__version__,
                matplotlib=matplotlib.__version__, python=platform.python_version(), platform=platform.platform())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--seeds", type=int, nargs="+")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    a = ap.parse_args()

    spec = json.load(open(a.config))
    base = dict(spec.get("base", {}))
    if a.duration:
        base["duration"] = a.duration
    seeds = a.seeds or spec.get("seeds", [1])
    out = os.path.join(a.out, spec["name"])
    os.makedirs(out, exist_ok=True)

    refs, rows, runs = {}, [], {}
    for var in variants(spec):
        label = var["label"]
        cfg = {**base, **{k: v for k, v in var.items() if k != "label"}}
        n = cfg.get("n", cosim.DEFAULTS["n"])
        if n not in refs:
            refs[n] = os.path.join(out, f"_reference_n{n}")
            cosim.run({**cfg, "mode": "ideal"}, refs[n])
        for s in seeds:
            d = os.path.join(out, label, f"seed{s}")
            full = cosim.run({**cfg, "seed": s}, d)
            m = metrics.compute(d, refs[n])
            rows.append({"label": label, "seed": s, **{k: full[k] for k in sorted(full)}, **m})
            runs.setdefault(label, []).append(d)
            print(f"{label:>24} seed={s}  pdr={m.get('pdr', 1):.3f}  lat={m.get('lat_mean_ms', 0):.1f}ms  "
                  f"aoi={m.get('aoi_mean_s', 0):.3f}s  gap_err={m['gap_err_mean_m']:.3f}m", flush=True)

    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    json.dump({"spec": spec, "seeds": seeds, "base_effective": {**cosim.DEFAULTS, **base}, "versions": versions()},
              open(os.path.join(out, "meta.json"), "w"), indent=1)
    plots.make_all(out, rows, runs, refs)
    print(f"results in {out}")


if __name__ == "__main__":
    main()
