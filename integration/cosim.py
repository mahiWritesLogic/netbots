"""Run one experiment: MuJoCo robots + network (ideal | ns-3 trace replay | ns-3 coupled).

trace   : ns-3 runs first (batch mode) and fixes every packet's fate; Python then replays
          it, binding the sender's real MuJoCo state at t_gen. Faithful only when send times
          and packet fate do not depend on robot state (true for `periodic` on this channel).
coupled : ns-3 runs in lockstep (epochs of EPOCH_NS) and carries the real state in the
          payload. Required for state-dependent policies such as `event`.
ideal   : no network; followers see ground truth. This is the coordination reference.
"""
import csv
import glob
import json
import math
import os
import subprocess

from integration.scheduler import MessageQueue, Receiver, load_trace
from simulation.robots import DT, Platoon, World, gap_errors

DT_NS = round(DT * 1e9)
EPOCH_NS = 10_000_000          # coupled-mode lockstep / policy decision granularity
LOG_EVERY = 10                 # robots.csv row every 10 ticks (20 ms)
NS3_DIR = os.environ.get("NS3_DIR", os.path.expanduser("~/ns-3.48"))

DEFAULTS = dict(n=3, duration=60.0, mode="trace", policy="periodic", rateHz=10.0, offsetMs=10.0,
                delayMs=0.1, jitterMs=0.0, loss=0.0, bgKbps=0.0, linkMbps=2.0, queuePkts=100,
                maxAgeMs=1000.0, eventDist=0.1, eventAngleDeg=10.0, heartbeatMs=1000.0, seed=1)
NS3_ARGS = ("n", "rateHz", "offsetMs", "delayMs", "jitterMs", "loss", "bgKbps", "linkMbps", "queuePkts")


def ns3_binary():
    hits = glob.glob(os.path.join(NS3_DIR, "build", "scratch", "ns3*-netbots*"))
    if not hits:
        raise FileNotFoundError(f"netbots ns-3 binary not found under {NS3_DIR}; run setup.sh")
    return hits[0]


def ns3_cmd(cfg, out, coupled):
    args = [ns3_binary(), f"--out={out}", "--seed=1", f"--run={cfg['seed']}",
            f"--duration={cfg['duration']}"] + [f"--{k}={cfg[k]}" for k in NS3_ARGS]
    return args + (["--coupled=1"] if coupled else [])


class Periodic:
    def __init__(self, cfg):
        self.period = round(1e9 / cfg["rateHz"])
        self.offset = round(cfg["offsetMs"] * 1e6)
        assert self.period % DT_NS == 0 and self.offset % DT_NS == 0, "send times must hit physics ticks"

    def due(self, i, t, state):
        o = i * self.offset
        return t >= o and (t - o) % self.period == 0


class EventTriggered:
    """Send when own pose drifted from the last sent one, or on heartbeat."""

    def __init__(self, cfg):
        self.d, self.a = cfg["eventDist"], math.radians(cfg["eventAngleDeg"])
        self.hb = round(cfg["heartbeatMs"] * 1e6)
        self.last = {}

    def due(self, i, t, s):
        prev = self.last.get(i)
        if prev is None or t - prev[0] >= self.hb or math.hypot(s[0] - prev[1][0], s[1] - prev[1][1]) > self.d \
                or abs((s[2] - prev[1][2] + math.pi) % (2 * math.pi) - math.pi) > self.a:
            self.last[i] = (t, s)
            return True
        return False


def run(cfg, outdir):
    cfg = {**DEFAULTS, **cfg}
    os.makedirs(outdir, exist_ok=True)
    n, mode = cfg["n"], cfg["mode"]
    world, ctrl = World(n), Platoon(n)
    max_age = None if cfg["maxAgeMs"] is None else round(cfg["maxAgeMs"] * 1e6)
    rx = [Receiver(max_age) for _ in range(n)]
    queue = MessageQueue()
    ticks = round(cfg["duration"] / DT)
    trace_path = os.path.join(outdir, "trace.csv")

    bind, sent_state, proc, policy = {}, {}, None, None
    if mode == "trace":
        if cfg["policy"] != "periodic":
            raise ValueError("trace replay only supports state-independent policies (periodic); use mode=coupled")
        subprocess.run(ns3_cmd(cfg, trace_path, False), check=True)
        for r in load_trace(trace_path):
            if r["kind"] != "state":
                continue
            if r["event"] == "tx":
                assert r["t_ns"] % DT_NS == 0, "tx not aligned to physics tick"
                bind.setdefault(r["t_ns"], []).append((r["src"], r["seq"]))
            elif r["event"] == "rx":
                queue.push(r["t_ns"], (r["t_ns"], r["src"], r["dst"], r["seq"], r["t_gen_ns"], None))
    elif mode == "coupled":
        policy = (Periodic if cfg["policy"] == "periodic" else EventTriggered)(cfg)
        proc = subprocess.Popen(ns3_cmd(cfg, trace_path, True), stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, text=True, bufsize=1)
    elif mode != "ideal":
        raise ValueError(mode)

    rf = open(os.path.join(outdir, "robots.csv"), "w", newline="")
    df = open(os.path.join(outdir, "deliveries.csv"), "w", newline="")
    rw, dw = csv.writer(rf), csv.writer(df)
    rw.writerow(["t", "id", "x", "y", "theta", "v", "omega", "gap_err", "belief_err"])
    dw.writerow(["t_consume_ns", "t_rx_ns", "src", "dst", "seq", "t_gen_ns", "outcome"])

    for k in range(ticks):
        t = k * DT_NS
        states = [world.state(i) for i in range(n)]

        if mode == "trace":
            for src, seq in bind.pop(t, ()):
                sent_state[(src, seq)] = states[src]
        elif mode == "coupled" and t % EPOCH_NS == 0:
            for i in range(n):
                if policy.due(i, t, states[i]):
                    proc.stdin.write(f"TX {i} {t} " + " ".join(repr(float(v)) for v in states[i]) + "\n")
            proc.stdin.write(f"RUN {t + EPOCH_NS}\n")
            for line in iter(proc.stdout.readline, ""):
                if line.startswith("DONE"):
                    break
                p = line.split()
                queue.push(int(p[1]), (int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5]),
                                       tuple(map(float, p[6:11]))))

        for t_rx, src, dst, seq, t_gen, st in queue.pop_due(t):
            assert t_rx <= t
            st = st if st is not None else sent_state[(src, seq)]
            dw.writerow([t, t_rx, src, dst, seq, t_gen, rx[dst].offer(t, src, seq, t_gen, st)])

        if mode == "ideal":
            beliefs = [None] + states[:-1]
        else:
            beliefs = [None] + [rx[i].get(t, i - 1) for i in range(1, n)]
        world.step(ctrl.commands(states, beliefs))

        if k % LOG_EVERY == 0:
            ge = [0.0] + gap_errors(states)
            for i, s in enumerate(states):
                b = beliefs[i]
                be = "" if i == 0 or b is None else math.hypot(b[0] - states[i - 1][0], b[1] - states[i - 1][1])
                rw.writerow([t / 1e9, i, *(f"{v:.5f}" for v in s), f"{ge[i]:.5f}", be if be == "" else f"{be:.5f}"])

    if proc:
        proc.stdin.write(f"RUN {ticks * DT_NS + 5_000_000_000}\n")   # drain in-flight packets
        for line in iter(proc.stdout.readline, ""):
            if line.startswith("DONE"):
                break
        proc.stdin.write("END\n")
        proc.stdin.close()
        proc.stdout.close()
        proc.wait()
    rf.close()
    df.close()
    with open(os.path.join(outdir, "config.json"), "w") as f:
        json.dump(cfg, f, indent=1)
    return cfg
