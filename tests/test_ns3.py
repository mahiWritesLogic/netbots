import filecmp
import os
import subprocess
import tempfile
import unittest

from integration import cosim
from integration.scheduler import load_trace
from tests.test_integration import have_ns3


def batch(d, name, **kw):
    cfg = {**cosim.DEFAULTS, "duration": 20.0, **kw}
    out = os.path.join(d, name)
    subprocess.run(cosim.ns3_cmd(cfg, out, False), check=True)
    return out


@unittest.skipUnless(have_ns3(), "ns-3 netbots binary not built")
class TestNs3(unittest.TestCase):
    def test_trace_valid(self):
        with tempfile.TemporaryDirectory() as d:
            rows = load_trace(batch(d, "t.csv", delayMs=20, jitterMs=10, loss=0.2))
        tx = {(r["src"], r["seq"]): r for r in rows if r["event"] == "tx"}
        rx = [r for r in rows if r["event"] == "rx"]
        drops = [r for r in rows if r["event"] == "drop"]
        self.assertEqual(len(tx), 3 * 200)                       # 3 robots x 10 Hz x 20 s
        for r in rx:
            t = tx[(r["src"], r["seq"])]                          # every rx has its tx
            self.assertEqual(r["t_gen_ns"], t["t_ns"])            # payload carries generation time
            self.assertNotEqual(r["src"], r["dst"])
            self.assertGreaterEqual(r["t_ns"] - r["t_gen_ns"], 20_000_000)
            self.assertLessEqual(r["t_ns"] - r["t_gen_ns"], 31_000_000)
        expected = len(tx) * 2
        self.assertEqual(len(rx) + len(drops), expected)          # nothing silently vanishes
        self.assertAlmostEqual(len(drops) / expected, 0.2, delta=0.04)
        self.assertTrue(all(r["reason"] == "rx_error" for r in drops))

    def test_reproducible_and_seed_sensitive(self):
        with tempfile.TemporaryDirectory() as d:
            kw = dict(jitterMs=50, loss=0.1, bgKbps=500)
            a, b = batch(d, "a.csv", **kw), batch(d, "b.csv", **kw)
            c = batch(d, "c.csv", seed=2, **kw)
            self.assertTrue(filecmp.cmp(a, b, shallow=False))
            self.assertFalse(filecmp.cmp(a, c, shallow=False))

    def test_overload_causes_queue_drops(self):
        with tempfile.TemporaryDirectory() as d:
            rows = load_trace(batch(d, "q.csv", bgKbps=2500, linkMbps=2))
        self.assertTrue(any(r["event"] == "drop" and r["reason"] == "queue" and r["kind"] == "state" for r in rows))


if __name__ == "__main__":
    unittest.main()
