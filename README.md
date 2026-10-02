# netbots

A simulation-first testbed for studying how network impairments affect multi-robot coordination.

- **MuJoCo 3.14.0** simulates three or more differential-drive robots in a warehouse.
- **ns-3.48** simulates the network that carries their state broadcasts.
- **Python** connects them, computes metrics and draws plots.

The network is the research subject. Physics and control are kept deliberately simple.

## Setup (Ubuntu 24.04, native or WSL2)

ns-3 officially targets Linux and macOS. On Windows, run everything inside WSL2 Ubuntu 24.04.

```bash
sudo apt-get install -y g++ cmake ninja-build python3-venv python3-dev   # one-time, needs sudo
bash setup.sh        # downloads ns-3.48, builds only core/network/internet/applications (+deps),
                     # copies network/netbots.cc into ns-3 scratch/, creates ~/.venvs/netbots
```

- Override the locations with `NS3_DIR` (default `~/ns-3.48`) and `VENV` (default `~/.venvs/netbots`).
- Pinned Python dependencies are listed in `requirements.txt`: mujoco, numpy, matplotlib, and nothing else.
- Re-run `bash setup.sh` after editing `network/netbots.cc`.

## Reproduce the baseline

```bash
PY=~/.venvs/netbots/bin/python
$PY -m unittest discover -s tests -t .                        # 7 tests
$PY experiments/run.py experiments/configs/baseline.json      # ~35 s, 5 seeds x 60 s
$PY analysis/summarize.py                                     # tables for every results/<exp>/summary.csv
bash experiments/run_all.sh                                   # all 7 experiments, ~15 min
# quick variants:  ... run.py <config> --seeds 1 --duration 20
```

The expected baseline result, identical for every seed, is:

| PDR | latency | AoI | gap error |
|---|---|---|---|
| 1.000 | 0.436 ms | 0.052 s | 0.065 m |

## Layout

| path | role |
|---|---|
| `simulation/robots.py` | MJCF generator (floor, visual-only shelves, N robots), ground-truth state, platoon controller. No networking code. |
| `network/netbots.cc` | ns-3 scenario: batch-trace mode and coupled (stdin/stdout lockstep) mode. |
| `integration/scheduler.py` | Trace loader, arrival-time message queue, per-robot receiver (duplicate, out-of-order and stale handling). |
| `integration/cosim.py` | One run: `ideal`, `trace` (replay) or `coupled`, with policies `periodic` and `event`. |
| `experiments/configs/*.json` | baseline plus one-factor sweeps (`latency`, `loss`, `jitter`, `load`, `robots`) and `policy`. |
| `experiments/run.py`, `run_all.sh` | Entry points. |
| `analysis/metrics.py`, `plots.py`, `summarize.py` | Metrics, figures, tables. |
| `results/<exp>/` | `summary.csv` (one row per variant×seed with the full config), `meta.json` (spec, effective config, ns-3/MuJoCo/numpy/matplotlib/Python versions), `*.png`, and per run `<variant>/seed<k>/{config.json, trace.csv, deliveries.csv, robots.csv}`. |

## Robots

- **Robot model:** a box chassis on a freejoint, two hinge wheels driven by MuJoCo `velocity` actuators, and two frictionless casters. Physics dt is 2 ms with the `implicitfast` integrator. Shelves have collisions disabled (they are scenery), so the physics stays trivial.
- **Task, a leader-follower platoon:** robot 0 loops through 4 waypoints at 0.5 m/s. Robot i>0 chases its *belief* of robot i−1 and holds a 1.0 m gap using v = v_pred + 1.2·(dist − 1.0) plus a heading P-controller. The predecessor's speed comes from the message.
- **No network inside the controller:** it only sees a `beliefs` list, so network conditions never touch the physics.

## Network model (ns-3)

- **Nodes:** N ns-3 nodes, each with UDP/IPv4 on a `SimpleNetDevice`, all attached to one `JitterChannel`. `JitterChannel` is an approximately 25-line subclass of ns-3's `SimpleChannel`.
- **Updates:** robots broadcast 56 B state updates at 10 Hz. Robot i starts at i·10 ms. The payload holds sender id, seq, t_gen (ns) and x, y, θ, v, ω. The receiver parses the payload; a packet tag only identifies packets inside drop traces.

| impairment | ns-3 mechanism |
|---|---|
| latency | channel base delay |
| jitter | extra per-receiver, per-packet delay ~ U(0, jitterMs) drawn in the channel; reordering happens when jitter exceeds the send period |
| loss | `RateErrorModel` (packet unit) on every receiving device, traced through `PhyRxDrop` |
| traffic load | per-robot CBR `OnOffApplication` (1000 B UDP broadcast) on the same interface: 2 Mbps device `DataRate`, then a traced `FifoQueueDisc` (`--queuePkts`, default 100) |

- **Trace** (`trace.csv`): `event{tx,rx,drop},t_ns,src,dst,seq,t_gen_ns,bytes,kind{state,bg},reason{rx_error,queue}`.
- **Seeds:** `RngSeedManager` seed is 1 and run = experiment seed. The same arguments give a byte-identical trace (tested).
- **Hidden-buffer fix:** `Ipv4AddressHelper::Assign` silently installs a 1000-packet pfifo_fast qdisc when none exists. The scenario installs its own traced FIFO qdisc and a 1-packet device queue, so all queueing is visible.
- **No MAC contention:** `SimpleChannel` models no shared-medium contention, no collisions and no PHY. Each node has its own rate-limited transmit path, which behaves like a switched LAN. Load therefore creates queueing at the *sender's* interface, not channel contention. Use this as a controlled impairment model, not as a Wi-Fi model.
- **Duplicates:** the channel never duplicates packets. Duplicate handling is exercised by a unit test with synthetic input.

## Integration and clocks

- **Clock mapping:** MuJoCo and ns-3 both start at t=0 and advance 1:1 in seconds. All bookkeeping is integer nanoseconds. Physics tick k is at t = k·2 ms, and send times must fall on ticks (asserted).
- **Delivery rule:** a message with ns-3 arrival time t_rx is consumed at the first tick ≥ t_rx, never before. `deliveries.csv` logs both times, and quantization is under 2 ms (tested).
- **Receiver:**
  - It discards duplicates (same src, seq).
  - It discards out-of-order messages (seq ≤ newest applied).
  - It always applies the newest message.
  - It withholds a belief older than `maxAgeMs` (1 s) from the controller, so the follower holds still instead of chasing a ghost.
- **`trace` mode (offline replay):** ns-3 runs first and fixes every packet's fate. Python then replays it and binds the sender's real MuJoCo state at t_gen. This is faithful only when send times and packet fate are independent of robot state, which holds for `periodic` on this position-independent channel.
- **Replay limitation:** replay *cannot* represent state-dependent sending (event-triggered policies) or position-dependent channels (e.g. Wi-Fi propagation).
- **`coupled` mode (the minimal event-driven coupling):** for those cases ns-3 runs in lockstep over a pipe in 10 ms epochs.
  1. Python sends `TX` lines (real state in the payload).
  2. Python sends `RUN t+10ms`.
  3. ns-3 returns every `RX` with an exact arrival time.
  4. Python steps physics through the epoch.

  Send decisions are made at epoch boundaries.
- **Cross-check:** with the same seed, periodic traffic gives byte-identical ns-3 traces and identical deliveries in `trace` and `coupled` modes (`tests/test_integration.py`).

## Metrics (`analysis/metrics.py`)

- **Latency:** t_rx − t_gen per received state packet (mean, p50, p95, max).
- **PDR:** rx / (tx·(N−1)). Drop rates are split by reason. Each queue drop counts once per receiver it would have reached.
- **AoI:** time-average over [first update, end] of `t − t_gen(newest applied)` at each receiver's controller, computed exactly from the sawtooth and then averaged over all (receiver, source) pairs.
- **Overhead:** state packets/s and bytes/s at the payload and IP layers (+28 B UDP/IPv4).
- **Coordination error:**
  - gap error |‖p_i − p_{i−1}‖ − 1 m| over followers, from MuJoCo ground truth, after a 5 s warm-up;
  - the excess over the **reference**: the same scenario with an ideal network, where followers read the true predecessor state every tick (`_reference_n<N>/`);
  - `dev_from_ref`: mean distance between each follower and its own position in the reference run.

## Results (5 seeds × 60 s each; mean, std across seeds in `summary.csv`)

These are measured on this machine (`analysis/summarize.py` prints them). Configurations without random elements are identical across seeds: delay-only, load-only, robot count and baseline.

| sweep | PDR | latency mean | AoI | gap error (excess over ideal) |
|---|---|---|---|---|
| baseline | 1.00 | 0.44 ms | 0.052 s | 0.065 m (+0.006) |
| delay 50 / 200 / 400 ms | 1.00 | 50 / 200 / 400 ms | 0.10 / 0.25 / 0.45 s | 0.097 / 0.143 / 0.232 m |
| loss 0.2 / 0.4 / 0.8 | 0.80 / 0.60 / 0.20 | 0.44 ms | 0.077 / 0.12 / 0.44 s | 0.081 / 0.103 / 0.289 m |
| jitter 50 / 200 / 400 ms | 1.00 | 25 / 99 / 198 ms | 0.076 / 0.146 / 0.216 s | 0.116 / 0.131 / 0.162 m |
| load 1800 / 1950 / 2100 kbps per robot | 1.00 / 1.00 / 0.77 | 2.1 / 172 / 381 ms | 0.053 / 0.22 / 0.47 s | 0.070 / 0.163 / 0.263 m |
| policy: periodic vs event (baseline) | 1.00 / 1.00 | 0.44 ms | 0.052 / 0.116 s | 0.065 / 0.112 m, with event using **46 % of the traffic** (1150 vs 2520 B/s) |
| policy: periodic vs event (loss 0.3) | 0.70 / 0.69 | 0.44 ms | 0.096 / 0.22 s | 0.091 / 0.152 m |

### Observations

- **Load cliff:** latency jumps between 1800 and 1950 kbps of background load, even though 1950 < 2000 kbps. The 28 B of IP/UDP header per 1000 B packet plus the state traffic push offered load above link capacity (about 2.01 Mbps), so the FIFO grows until it overflows. Queue drops then dominate (PDR 0.77 at 2100 kbps).
- **Jitter reordering:** jitter ≥ 200 ms (more than the 100 ms period) causes out-of-order arrivals, which the receiver discards (453 and 1210 per run).
- **Event-triggered policy:** the 0.1 m / 10° / 1 s heartbeat policy halves the traffic but doubles AoI and gap error in this task. Its sparse updates do not survive loss as well.
- **Non-monotone and unexplained results:**
  - Coordination error is not strictly monotone in delay: 10 ms gives 0.107 m while 50 ms gives 0.097 m.
  - In the robot-count sweep, excess gap error is *negative* for N = 5 and 8. The ideal-information reference itself has larger gap error with long platoons, while `dev_from_ref` grows with N (0.27 m at N = 8).

  Neither effect has been investigated. Treat them as properties of this simple controller, not general findings.

## Limitations

- No MAC contention or PHY model (see Network model). Load creates sender-side queueing only.
- Coupled mode makes send decisions every 10 ms, and all deliveries are consumed at 2 ms tick resolution.
- Background CBR traffic is deterministic, so load-only sweeps show zero variance across seeds.
- Shelves have no collisions and there is no obstacle avoidance. The controller is a simple pursuit law with no extrapolation of stale beliefs.
- MuJoCo runs headless. Trajectories are plotted with matplotlib (`trajectories.png`) instead of rendered.
