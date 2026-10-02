"""Arrival-time message scheduling between ns-3 traces and robot controllers.

Clock mapping: MuJoCo time and ns-3 time are both "seconds since t=0" and advance 1:1.
All bookkeeping is integer nanoseconds. Physics tick k happens at t = k * DT_NS; a message
with arrival time t_rx is consumed at the first tick with t >= t_rx, never earlier.
"""
import csv
import heapq
import itertools

INT_COLS = ("t_ns", "src", "dst", "seq", "t_gen_ns", "bytes")


def load_trace(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for c in INT_COLS:
            r[c] = int(r[c])
    return rows


class MessageQueue:
    """Min-heap of in-flight messages keyed by arrival time."""

    def __init__(self):
        self._h, self._tie = [], itertools.count()

    def push(self, t_rx_ns, msg):
        heapq.heappush(self._h, (t_rx_ns, next(self._tie), msg))

    def pop_due(self, now_ns):
        while self._h and self._h[0][0] <= now_ns:
            yield heapq.heappop(self._h)[2]

    def __len__(self):
        return len(self._h)


class Receiver:
    """Per-robot inbox filter. Keeps only the freshest state per source.

    duplicate     (src, seq) already seen                          -> discarded
    out_of_order  seq older than the newest already applied         -> discarded
    stale         newest so far but already older than max_age_ns   -> applied (still the best info)
    applied       newest so far                                     -> becomes the belief about src
    A belief older than max_age_ns is withheld from the controller (`get` returns None),
    so a robot holds still instead of chasing a ghost.
    """

    def __init__(self, max_age_ns=None):
        self.max_age_ns = max_age_ns
        self.seen = set()
        self.last_seq = {}
        self.belief = {}  # src -> (t_gen_ns, state)

    def _too_old(self, now_ns, t_gen_ns):
        return self.max_age_ns is not None and now_ns - t_gen_ns > self.max_age_ns

    def offer(self, now_ns, src, seq, t_gen_ns, state):
        if (src, seq) in self.seen:
            return "duplicate"
        self.seen.add((src, seq))
        if seq <= self.last_seq.get(src, -1):
            return "out_of_order"
        self.last_seq[src] = seq
        self.belief[src] = (t_gen_ns, state)
        return "stale" if self._too_old(now_ns, t_gen_ns) else "applied"

    def get(self, now_ns, src):
        b = self.belief.get(src)
        return None if b is None or self._too_old(now_ns, b[0]) else b[1]
