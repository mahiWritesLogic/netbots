import math
import unittest

from simulation.robots import DT, Platoon, World, gap_errors


class TestSim(unittest.TestCase):
    def test_three_robots_ideal_platoon(self):
        w, p = World(3), Platoon(3)
        errs = []
        for k in range(int(20 / DT)):
            s = [w.state(i) for i in range(3)]
            w.step(p.commands(s, [None] + s[:-1]))
            if k * DT > 5:
                errs.append(gap_errors(s))
        s = [w.state(i) for i in range(3)]
        self.assertTrue(all(math.isfinite(v) for st in s for v in st))
        self.assertGreaterEqual(p.wp, 2, "leader should pass at least two waypoints in 20 s")
        for i, st in enumerate(s):
            self.assertGreater(math.hypot(st[0] - (3.0 - i), st[1] + 2.5), 2.0, f"robot {i} did not move")
        self.assertLess(max(max(e) for e in errs), 0.5)


if __name__ == "__main__":
    unittest.main()
