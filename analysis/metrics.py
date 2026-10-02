"""Per-run metrics from trace.csv (ns-3), deliveries.csv and robots.csv (Python/MuJoCo)."""
import csv
import json
import os
from collections import defaultdict

import numpy as np

from integration.scheduler import load_trace

WARMUP_S = 5.0          # coordination metrics ignore the start-up transient
IP_UDP_HDR = 28         # bytes added below the 56 B state payload (IPv4 20 + UDP 8)


def _csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def latencies_ms(run_dir):
    return np.array([(r["t_ns"] - r["t_gen_ns"]) / 1e6 for r in load_trace(os.path.join(run_dir, "trace.csv"))
                     if r["event"] == "rx" and r["kind"] == "state"])


def mean_aoi_s(deliveries, n, t_end_ns):
    """Time-average Age of Information per (receiver, source), averaged over pairs.

    Age at the receiver's controller: AoI(t) = t - t_gen of the newest applied update,
    updated at consumption time. Integrated exactly over [first update, t_end].
    """
    ups = defaultdict(list)
    for r in deliveries:
        if r["outcome"] in ("applied", "stale"):
            ups[(int(r["dst"]), int(r["src"]))].append((int(r["t_consume_ns"]), int(r["t_gen_ns"])))
    vals = []
    for pair in ((d, s) for d in range(n) for s in range(n) if d != s):
        u = ups.get(pair)
        if not u:
            continue
        area, (a, g) = 0.0, u[0]
        for b, g2 in u[1:] + [(t_end_ns, None)]:
            area += ((b - g) ** 2 - (a - g) ** 2) / 2
            a, g = b, (g2 if g2 is not None else g)
        vals.append(area / (t_end_ns - u[0][0]) / 1e9)
    return float(np.mean(vals)) if vals else float("nan"), n * (n - 1) - len(vals)


def coordination(run_dir):
    """Follower rows after warm-up: gap error and belief error arrays, plus positions keyed by (t, id)."""
    gap, bel, pos = [], [], {}
    for r in _csv(os.path.join(run_dir, "robots.csv")):
        t, i = float(r["t"]), int(r["id"])
        pos[(round(t, 3), i)] = (float(r["x"]), float(r["y"]))
        if i == 0 or t < WARMUP_S:
            continue
        gap.append(float(r["gap_err"]))
        if r["belief_err"]:
            bel.append(float(r["belief_err"]))
    return np.array(gap), np.array(bel), pos


def compute(run_dir, ref_dir=None):
    cfg = json.load(open(os.path.join(run_dir, "config.json")))
    n, dur = cfg["n"], cfg["duration"]
    m = {}

    if cfg["mode"] != "ideal":
        tr = load_trace(os.path.join(run_dir, "trace.csv"))
        st = [r for r in tr if r["kind"] == "state"]
        tx = [r for r in st if r["event"] == "tx"]
        rx = [r for r in st if r["event"] == "rx"]
        rx_err = sum(r["event"] == "drop" and r["reason"] == "rx_error" for r in st)
        q_drop = sum(r["event"] == "drop" and r["reason"] == "queue" for r in st)
        expected = len(tx) * (n - 1)
        lat = np.array([(r["t_ns"] - r["t_gen_ns"]) / 1e6 for r in rx])
        m.update(
            tx_state=len(tx), rx_state=len(rx), expected_rx=expected,
            pdr=len(rx) / expected if expected else float("nan"),
            drop_rate_rx_error=rx_err / expected if expected else float("nan"),
            drop_rate_queue=q_drop * (n - 1) / expected if expected else float("nan"),
            undelivered=expected - len(rx) - rx_err - q_drop * (n - 1),
            lat_mean_ms=lat.mean() if len(lat) else float("nan"),
            lat_p50_ms=float(np.percentile(lat, 50)) if len(lat) else float("nan"),
            lat_p95_ms=float(np.percentile(lat, 95)) if len(lat) else float("nan"),
            lat_max_ms=lat.max() if len(lat) else float("nan"),
            bg_drops=sum(r["event"] == "drop" and r["kind"] == "bg" for r in tr),
            overhead_pkts_per_s=len(tx) / dur,
            overhead_payload_Bps=sum(r["bytes"] for r in tx) / dur,
            overhead_ip_Bps=sum(r["bytes"] + IP_UDP_HDR for r in tx) / dur,
        )
        dl = _csv(os.path.join(run_dir, "deliveries.csv"))
        for o in ("applied", "stale", "out_of_order", "duplicate"):
            m[f"msg_{o}"] = sum(r["outcome"] == o for r in dl)
        m["aoi_mean_s"], m["aoi_pairs_never_updated"] = mean_aoi_s(dl, n, round(dur * 1e9))

    gap, bel, pos = coordination(run_dir)
    m.update(gap_err_mean_m=gap.mean(), gap_err_rms_m=float(np.sqrt((gap ** 2).mean())), gap_err_max_m=gap.max(),
             belief_err_mean_m=bel.mean() if len(bel) else float("nan"))
    if ref_dir:
        rgap, _, rpos = coordination(ref_dir)
        m["gap_err_excess_m"] = m["gap_err_mean_m"] - rgap.mean()
        dev = [np.hypot(p[0] - rpos[k][0], p[1] - rpos[k][1]) for k, p in pos.items()
               if k[1] > 0 and k[0] >= WARMUP_S and k in rpos]
        m["dev_from_ref_mean_m"] = float(np.mean(dev))
    return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in m.items()}
