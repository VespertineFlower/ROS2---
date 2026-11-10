"""Unit tests for the pure pursuit path tracking law."""

import math
import unittest

from maze_control.control_law import (DEFAULT_PP_CFG, path_length,
                                      pure_pursuit_command, smooth_path)


def cfg(**kwargs):
    c = dict(DEFAULT_PP_CFG)
    c.update(kwargs)
    return c


class TestPurePursuit(unittest.TestCase):

    def test_straight_path_no_steering(self):
        path = [(0.0, 0.0), (2.0, 0.0)]
        v, w, carrot, _, remaining, alpha = pure_pursuit_command(
            0.2, 0.0, 0.0, path, cfg())
        self.assertGreater(v, 0.0)
        self.assertAlmostEqual(w, 0.0, places=6)
        self.assertGreater(carrot[0], 0.2)
        self.assertGreater(remaining, 1.5)

    def test_offset_above_path_steers_back(self):
        # robot is 0.15 m to the left (positive y) of a path along +x
        path = [(0.0, 0.0), (2.0, 0.0)]
        _, w, _, _, _, alpha = pure_pursuit_command(
            0.2, 0.15, 0.0, path, cfg())
        self.assertLess(alpha, 0.0)
        self.assertLess(w, 0.0)          # steer right to come back down

    def test_offset_below_path_steers_left(self):
        path = [(0.0, 0.0), (2.0, 0.0)]
        _, w, _, _, _, alpha = pure_pursuit_command(
            0.2, -0.15, 0.0, path, cfg())
        self.assertGreater(alpha, 0.0)
        self.assertGreater(w, 0.0)

    def test_heading_opposite_rotates(self):
        path = [(0.0, 0.0), (2.0, 0.0)]
        _, w, _, _, _, alpha = pure_pursuit_command(
            0.2, 0.0, math.pi, path, cfg())
        self.assertAlmostEqual(abs(alpha), math.pi, places=3)
        self.assertNotEqual(w, 0.0)

    def test_goal_stop(self):
        path = [(0.0, 0.0), (1.0, 0.0)]
        v, w, _, _, remaining, _ = pure_pursuit_command(
            0.97, 0.0, 0.0, path, cfg(goal_tolerance=0.06))
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.0)
        self.assertAlmostEqual(remaining, 0.0, places=6)

    def test_slowdown_near_goal(self):
        path = [(0.0, 0.0), (1.0, 0.0)]
        v_far, _, _, _, _, _ = pure_pursuit_command(
            0.1, 0.0, 0.0, path, cfg())
        v_near, _, _, _, _, _ = pure_pursuit_command(
            0.75, 0.0, 0.0, path, cfg())
        self.assertLess(v_near, v_far)

    def test_path_progress_index_moves_forward(self):
        path = [(0.0, 0.0), (1.0, 0.0)]
        _, _, _, index, _, _ = pure_pursuit_command(0.5, 0.0, 0.0, path, cfg())
        self.assertEqual(index, 0)
        self.assertGreater(path_length(path), 0.99)

    def test_smooth_path_keeps_ends_and_rounds_corner(self):
        raw = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        pts = smooth_path(raw, radius=0.2, samples=6)
        self.assertEqual(pts[0], raw[0])
        self.assertEqual(pts[-1], raw[-1])
        self.assertGreater(len(pts), len(raw))
        self.assertLess(path_length(pts), path_length(raw))
        # the corner (1,0) must not be part of the smoothed path anymore
        for x, y in pts:
            self.assertFalse(abs(x - 1.0) < 1e-9 and abs(y) < 1e-9)

    def test_smooth_path_short_path_is_unchanged(self):
        raw = [(0.0, 0.0), (1.0, 0.0)]
        self.assertEqual(smooth_path(raw), raw)

    def test_curvature_sign_follows_carrot_side(self):
        path = [(0.0, 0.0), (1.0, 1.0)]
        _, w_left, _, _, _, _ = pure_pursuit_command(0.0, 0.0, 0.0, path, cfg())
        self.assertGreater(w_left, 0.0)   # path bends to the left


if __name__ == '__main__':
    unittest.main()
