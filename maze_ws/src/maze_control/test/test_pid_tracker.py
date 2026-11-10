"""Offline tests for the feedforward + PID path follower (kinematic sim)."""

import math
import unittest

import numpy as np

from maze_control.pid_tracker import PidPathFollower, project, path_geometry


def simulate(follower, x, y, yaw, steps=400, dt=0.05, v_scale=1.0, **kw):
    """Integrate a unicycle with the controller (no Gazebo needed)."""
    traj = []
    for _ in range(steps):
        v, w, state = follower.update(x, y, yaw, dt=dt, **kw)
        x += v * v_scale * math.cos(yaw) * dt
        y += v * v_scale * math.sin(yaw) * dt
        yaw += w * dt
        traj.append((x, y, yaw, state))
        if follower.finished:
            break
    return traj


class TestProjection(unittest.TestCase):
    def test_signed_lateral_error(self):
        geom = path_geometry([(0.0, 0.0), (2.0, 0.0)])
        s, e, heading, kappa = project(geom, 0.5, 0.2)
        self.assertAlmostEqual(s, 0.5, places=2)
        self.assertGreater(e, 0.1)          # robot is left of an east path
        self.assertAlmostEqual(heading, 0.0, places=6)
        s, e, _, _ = project(geom, 0.5, -0.2)
        self.assertLess(e, -0.1)

    def test_curvature_of_a_left_turn_is_positive(self):
        pts = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3)]
        geom = path_geometry(pts, {'curv_smooth': 1})
        self.assertGreater(geom['kappa'][1], 0.0)


class TestTracking(unittest.TestCase):
    def test_converges_to_a_straight_line(self):
        path = [(0.0, 0.0), (3.0, 0.0)]
        follower = PidPathFollower(path, speeds=[0.3, 0.3])
        traj = simulate(follower, 0.0, 0.30, 0.0, steps=300)
        final_e = abs(traj[min(120, len(traj) - 1)][3]['e'])
        self.assertLess(final_e, 0.03, 'lateral error did not converge')
        self.assertGreater(traj[-1][0], 2.0, 'robot did not travel down the path')

    def test_follows_an_arc(self):
        # quarter circle of radius 0.7, sampled finely
        rad = 0.7
        path = [(rad * math.sin(a), rad - rad * math.cos(a))
                for a in np.linspace(0.0, math.pi / 2, 40)]
        follower = PidPathFollower(path, speeds=np.full(len(path), 0.3))
        traj = simulate(follower, 0.0, 0.0, 0.0, steps=300)
        errs = [abs(st['e']) for (_, _, _, st) in traj[20:]]
        self.assertLess(max(errs), 0.08, 'arc tracking error too large')

    def test_speed_respects_the_slew_limits(self):
        path = [(0.0, 0.0), (3.0, 0.0)]
        follower = PidPathFollower(path, speeds=np.full(2, 0.4))
        v, w, _ = follower.update(0.0, 0.0, 0.0, dt=0.05)
        self.assertLessEqual(v, follower.cfg['accel'] * 0.05 + 1e-9)
        v2, _, _ = follower.update(0.0, 0.0, 0.0, dt=0.05)
        self.assertLessEqual(v2 - v, follower.cfg['accel'] * 0.05 + 1e-9)

    def test_stops_at_the_end_of_the_path(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.6)]
        follower = PidPathFollower(path, speeds=np.full(3, 0.3))
        simulate(follower, 0.0, 0.0, 0.0, steps=500)
        self.assertTrue(follower.finished)
        self.assertLess(follower.last['v'], 0.05)

    def test_projection_ignores_a_stale_hint_window(self):
        """A doubled-back route must not report the stale branch."""
        # east leg at y=0.0 then north leg at x=2.0; the two legs are 2 m apart
        path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
        geom = path_geometry(path)
        # hint points at the start of the path, but the robot is on the north leg
        s, e, heading, _k = project(geom, 2.05, 1.50, hint=0)
        self.assertLess(abs(e), 0.10, 'lateral error must use the near branch')
        self.assertAlmostEqual(s, 3.50, delta=0.10)
        self.assertAlmostEqual(math.degrees(heading), 90.0, delta=2.0)

    def test_projection_does_not_jump_when_the_hint_stales(self):
        path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
        geom = path_geometry(path)
        near = project(geom, 1.95, 1.20, hint=95)
        stale = project(geom, 1.95, 1.20, hint=0)
        self.assertAlmostEqual(near[1], stale[1], delta=0.02)


if __name__ == '__main__':
    unittest.main()
