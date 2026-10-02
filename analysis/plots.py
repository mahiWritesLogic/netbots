"""Experiment-level figures: mean +/- std across seeds per variant."""
import csv
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analysis.metrics import latencies_ms
from simulation.robots import WAYPOINTS


def _bars(ax, rows, labels, key, ylabel):
    by = defaultdict(list)
    for r in rows:
        by[r["label"]].append(r.get(key, np.nan))
    mu = [np.nanmean(by[lab]) for lab in labels]
    sd = [np.nanstd(by[lab]) for lab in labels]
    ax.bar(range(len(labels)), mu, yerr=sd, capsize=4, color="#4C72B0")
    ax.set_xticks(range(len(labels)), labels, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.3)


def _save(fig, out, name):
    fig.tight_layout()
    fig.savefig(os.path.join(out, name), dpi=120)
    plt.close(fig)


def make_all(out, rows, runs, refs):
    labels = list(dict.fromkeys(r["label"] for r in rows))
    netrows = [r for r in rows if "pdr" in r]

    if netrows:
        fig, ax = plt.subplots(figsize=(6, 4))
        for lab in labels:
            lat = np.sort(np.concatenate([latencies_ms(d) for d in runs[lab]]))
            if len(lat):
                ax.plot(lat, np.arange(1, len(lat) + 1) / len(lat), label=lab)
        ax.set_xscale("log")
        ax.set_xlabel("end-to-end latency [ms] (log)")
        ax.set_ylabel("CDF")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
        _save(fig, out, "latency_cdf.png")

        fig, axs = plt.subplots(1, 2, figsize=(10, 4))
        _bars(axs[0], netrows, labels, "pdr", "packet delivery ratio")
        _bars(axs[1], netrows, labels, "overhead_ip_Bps", "state traffic [B/s, IP layer]")
        _save(fig, out, "pdr_overhead.png")

        fig, ax = plt.subplots(figsize=(6, 4))
        _bars(ax, netrows, labels, "aoi_mean_s", "mean Age of Information [s]")
        _save(fig, out, "aoi.png")

    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    _bars(axs[0], rows, labels, "gap_err_mean_m", "mean gap error [m]")
    _bars(axs[1], rows, labels, "dev_from_ref_mean_m", "deviation from ideal-network run [m]")
    _save(fig, out, "coord_error.png")

    # Trajectories of the first run (seed) of the first variant vs. the reference.
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for src, style in ((next(iter(refs.values())), ":"), (runs[labels[0]][0], "-")):
        tr = defaultdict(list)
        with open(os.path.join(src, "robots.csv")) as f:
            for r in csv.DictReader(f):
                tr[int(r["id"])].append((float(r["x"]), float(r["y"])))
        for i, p in tr.items():
            p = np.array(p)
            ax.plot(p[:, 0], p[:, 1], style, lw=1, label=f"r{i} {'ideal' if style == ':' else labels[0]}")
    w = np.array(WAYPOINTS + WAYPOINTS[:1])
    ax.plot(w[:, 0], w[:, 1], "k+", ms=10)
    ax.set_aspect("equal")
    ax.legend(fontsize=6, ncol=2)
    _save(fig, out, "trajectories.png")
