"""MuJoCo warehouse with N differential-drive robots + a network-agnostic platoon controller.

Nothing in this file knows about networking: followers act on a `beliefs` dict
{robot_id: (x, y, theta, v, omega)} that the caller fills however it likes.
"""
import math
import mujoco
import numpy as np

DT = 0.002            # physics step [s]
WHEEL_R = 0.05        # wheel radius [m]
TRACK = 0.28          # wheel separation [m]
GAP = 1.0             # desired platoon spacing [m]
V_MAX = 0.5           # leader cruise speed [m/s]
# Leader loops counter-clockwise around the central shelf block.
WAYPOINTS = [(4.5, -2.5), (4.5, 2.5), (-4.5, 2.5), (-4.5, -2.5)]


def initial_pose(i):
    """Platoon starts in a line on the bottom aisle, heading +x, spaced by GAP."""
    return (3.0 - i * GAP, -2.5, 0.0)


def build_mjcf(n):
    robots = []
    for i in range(n):
        x, y, _ = initial_pose(i)
        robots.append(f"""
    <body name="r{i}" pos="{x} {y} {WHEEL_R}">
      <freejoint name="r{i}_free"/>
      <geom type="box" size="0.15 0.10 0.04" mass="2" rgba="{0.9 - 0.1 * (i % 8)} 0.3 {0.2 + 0.1 * (i % 8)} 1"/>
      <geom type="sphere" size="{WHEEL_R - 0.002}" pos="0.12 0 0" mass="0.05" friction="0 0 0" condim="1"/>
      <geom type="sphere" size="{WHEEL_R - 0.002}" pos="-0.12 0 0" mass="0.05" friction="0 0 0" condim="1"/>
      <body name="r{i}_wl" pos="0 {TRACK / 2} 0">
        <joint name="r{i}_left" type="hinge" axis="0 1 0" damping="0.01"/>
        <geom type="cylinder" size="{WHEEL_R} 0.02" euler="90 0 0" mass="0.2" friction="1.5 0.01 0.001"/>
      </body>
      <body name="r{i}_wr" pos="0 {-TRACK / 2} 0">
        <joint name="r{i}_right" type="hinge" axis="0 1 0" damping="0.01"/>
        <geom type="cylinder" size="{WHEEL_R} 0.02" euler="90 0 0" mass="0.2" friction="1.5 0.01 0.001"/>
      </body>
    </body>""")
    acts = "".join(
        f'\n    <velocity joint="r{i}_{s}" kv="2" ctrlrange="-25 25" forcerange="-3 3"/>'
        for i in range(n) for s in ("left", "right"))
    # Shelves are visual only (contype/conaffinity 0): routes avoid them, physics stays trivial.
    shelves = "".join(
        f'\n    <geom type="box" pos="{x} {y} 0.5" size="1.5 0.4 0.5" contype="0" conaffinity="0" rgba="0.6 0.5 0.3 1"/>'
        for x in (-2.5, 0.0, 2.5) for y in (-1.0, 1.0))
    return f"""<mujoco model="netbots_warehouse">
  <option timestep="{DT}" integrator="implicitfast"/>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="8 5 0.1" rgba="0.8 0.8 0.8 1"/>{shelves}{''.join(robots)}
  </worldbody>
  <actuator>{acts}
  </actuator>
</mujoco>"""


class World:
    def __init__(self, n):
        self.n = n
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(n))
        self.data = mujoco.MjData(self.model)
        self.qadr = [self.model.jnt_qposadr[self.model.joint(f"r{i}_free").id] for i in range(n)]
        self.vadr = [self.model.jnt_dofadr[self.model.joint(f"r{i}_free").id] for i in range(n)]
        mujoco.mj_forward(self.model, self.data)

    @property
    def t(self):
        return self.data.time

    def state(self, i):
        """(x, y, theta, v, omega) from ground-truth physics."""
        q = self.data.qpos[self.qadr[i]:self.qadr[i] + 7]
        w, qx, qy, qz = q[3:7]
        th = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        dv = self.data.qvel[self.vadr[i]:self.vadr[i] + 6]
        v = dv[0] * math.cos(th) + dv[1] * math.sin(th)   # freejoint linear vel is world-frame
        return (q[0], q[1], th, v, dv[5])                  # angular vel is body-frame; flat ground -> yaw rate

    def step(self, cmds):
        """cmds: list of (v, omega) body-velocity commands, one per robot."""
        for i, (v, om) in enumerate(cmds):
            self.data.ctrl[2 * i] = (v - om * TRACK / 2) / WHEEL_R      # left
            self.data.ctrl[2 * i + 1] = (v + om * TRACK / 2) / WHEEL_R  # right
        mujoco.mj_step(self.model, self.data)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Platoon:
    """Robot 0 tracks WAYPOINTS; robot i>0 chases its *belief* of robot i-1 at distance GAP."""

    def __init__(self, n):
        self.n = n
        self.wp = 0

    def leader_cmd(self, s):
        tx, ty = WAYPOINTS[self.wp]
        if math.hypot(tx - s[0], ty - s[1]) < 0.3:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            tx, ty = WAYPOINTS[self.wp]
        err = wrap(math.atan2(ty - s[1], tx - s[0]) - s[2])
        return V_MAX * max(math.cos(err), 0.0), float(np.clip(3.0 * err, -2, 2))

    @staticmethod
    def follower_cmd(s, target):
        if target is None:          # nothing heard yet: hold still
            return 0.0, 0.0
        dx, dy = target[0] - s[0], target[1] - s[1]
        dist = math.hypot(dx, dy)
        err = wrap(math.atan2(dy, dx) - s[2])
        # feed-forward the predecessor's broadcast speed + P on gap error
        v = float(np.clip(target[3] + 1.2 * (dist - GAP), -0.2, 0.8))
        if v > 0:
            v *= max(math.cos(err), 0.0)
        om = float(np.clip(3.0 * err, -2, 2)) if dist > 0.2 else 0.0
        return v, om

    def commands(self, states, beliefs):
        """states: own ground-truth states; beliefs[i]: what robot i believes about robot i-1."""
        return [self.leader_cmd(states[0])] + [
            self.follower_cmd(states[i], beliefs[i]) for i in range(1, self.n)]


def gap_errors(states):
    """Coordination error: |distance to predecessor - GAP| per follower, from ground truth."""
    return [abs(math.hypot(states[i][0] - states[i - 1][0], states[i][1] - states[i - 1][1]) - GAP)
            for i in range(1, len(states))]


if __name__ == "__main__":
    # Ideal-information run (followers see the truth): smoke test the physics + controller.
    w, p = World(3), Platoon(3)
    for k in range(int(30 / DT)):
        s = [w.state(i) for i in range(3)]
        w.step(p.commands(s, [None] + s[:-1]))
        if k % 2500 == 0:
            print(f"t={w.t:5.1f} " + " ".join(f"({x:+.2f},{y:+.2f},{v:.2f})" for x, y, _, v, _ in s),
                  "gap_err", [round(e, 2) for e in gap_errors(s)])
