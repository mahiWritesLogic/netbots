import csv
import filecmp
import os
import tempfile
import unittest

from integration import cosim
from integration.scheduler import MessageQueue, Receiver


def have_ns3():
    try:
        cosim.ns3_binary()
        return True
    except FileNotFoundError:
        return False


class TestScheduler(unittest.TestCase):
    def test_queue_releases_by_arrival_time_not_generation(self):
        q = MessageQueue()
        q.push(50, "late-gen-early-arrival")
        q.push(30, "a")
        q.push(90, "b")
        self.assertEqual(list(q.pop_due(29)), [])
        self.assertEqual(list(q.pop_due(50)), ["a", "late-gen-early-arrival"])
        self.assertEqual(list(q.pop_due(89)), [])
        self.assertEqual(list(q.pop_due(90)), ["b"])

    def test_receiver_duplicate_out_of_order_stale(self):
        r = Receiver(max_age_ns=100)
        self.assertEqual(r.offer(10, 0, 0, 0, "s0"), "applied")
        self.assertEqual(r.offer(20, 0, 2, 15, "s2"), "applied")
        self.assertEqual(r.offer(25, 0, 1, 12, "s1"), "out_of_order")   # older seq arrives later
        self.assertEqual(r.offer(26, 0, 2, 15, "s2"), "duplicate")
        self.assertEqual(r.get(30, 0), "s2")
        self.assertIsNone(r.get(200, 0))                               # belief aged out
        self.assertEqual(r.offer(300, 0, 3, 150, "s3"), "stale")        # newest, but old on arrival
        self.assertIsNone(r.get(300, 0))
        self.assertEqual(r.offer(300, 1, 0, 290, "x"), "applied")       # per-source sequence spaces


@unittest.skipUnless(have_ns3(), "ns-3 netbots binary not built")
class TestCosim(unittest.TestCase):
    cfg = dict(duration=10.0, delayMs=30, jitterMs=250, loss=0.1)

    def test_consumed_after_arrival_and_trace_matches_coupled(self):
        with tempfile.TemporaryDirectory() as d:
            a, b = os.path.join(d, "trace"), os.path.join(d, "coupled")
            cosim.run({**self.cfg, "mode": "trace"}, a)
            cosim.run({**self.cfg, "mode": "coupled"}, b)
            # Same seed + same send times => ns-3 must decide identical packet fates.
            self.assertTrue(filecmp.cmp(os.path.join(a, "trace.csv"), os.path.join(b, "trace.csv"), shallow=False))
            for run_dir in (a, b):
                with open(os.path.join(run_dir, "deliveries.csv")) as f:
                    rows = list(csv.DictReader(f))
                self.assertTrue(rows)
                for r in rows:
                    self.assertGreaterEqual(int(r["t_consume_ns"]), int(r["t_rx_ns"]))
                    self.assertLess(int(r["t_consume_ns"]) - int(r["t_rx_ns"]), cosim.DT_NS)
                outcomes = {r["outcome"] for r in rows}
                self.assertIn("out_of_order", outcomes, "jitter > send period must reorder")
            with open(os.path.join(a, "deliveries.csv")) as fa, open(os.path.join(b, "deliveries.csv")) as fb:
                self.assertEqual(fa.read(), fb.read())


if __name__ == "__main__":
    unittest.main()
