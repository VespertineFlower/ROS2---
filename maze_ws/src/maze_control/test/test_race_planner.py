"""Unit tests for the off-line race planner (two-pass strategy)."""

import os
import tempfile
import unittest

import numpy as np

from maze_control.race_planner import (DEFAULT_CFG, GridMap, astar,
                                       check_route, effective_curvature,
                                       maze_bbox, plan_race, resample,
                                       restrict_to_maze, shortcut, simplify,
                                       smooth_path, smoothing_pass,
                                       speed_profile)


def box(cx, cy, sx, sy, name='wall'):
    return (cx, cy, sx, sy, name)


def door_room():
    """4 x 4 room split by a wall at x = 2 with a 0.6 m door at y = 2."""
    walls = [
        box(2.0, 4.05, 4.2, 0.1, 'north'),
        box(2.0, -0.05, 4.2, 0.1, 'south'),
        box(-0.05, 2.0, 0.1, 4.2, 'west'),
        box(4.05, 2.0, 0.1, 4.2, 'east'),
        box(2.0, 0.85, 0.1, 1.7, 'split_south'),
        box(2.0, 3.15, 0.1, 1.7, 'split_north'),
    ]
    return walls


class TestGridMap(unittest.TestCase):
    def setUp(self):
        self.gmap = GridMap.from_walls(door_room())

    def test_walls_are_occupied_and_free_space_is_clear(self):
        self.assertGreater(self.gmap.clearance_at(2.0, 2.0), 0.2)
        self.assertLess(self.gmap.clearance_at(2.0, 3.0), 0.1)
        self.assertFalse(self.gmap.free(2.0, 3.0, 0.15))

    def test_save_and_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'map.npz')
            self.gmap.save(path)
            again = GridMap.load(path)
        self.assertTrue(np.array_equal(self.gmap.occupied, again.occupied))
        self.assertAlmostEqual(self.gmap.res, again.res)
        self.assertAlmostEqual(self.gmap.ox, again.ox)


class TestPlanning(unittest.TestCase):
    def setUp(self):
        self.gmap = GridMap.from_walls(door_room())

    def test_astar_goes_through_the_door(self):
        path = astar(self.gmap, (0.6, 2.0), (3.4, 2.0))
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 10)
        self.assertTrue(any(abs(p[0] - 2.0) < 0.4 and abs(p[1] - 2.0) < 0.4
                            for p in path))

    def test_shortcut_keeps_away_from_the_walls(self):
        raw = astar(self.gmap, (0.6, 0.6), (3.4, 3.4))
        self.assertIsNotNone(raw)
        pulled = shortcut(self.gmap, raw)
        need = DEFAULT_CFG['clearance_min'] + DEFAULT_CFG['obstacle_radius']
        for a, b in zip(pulled, pulled[1:]):
            steps = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
            for k in range(steps + 1):
                x = a[0] + (b[0] - a[0]) * k / steps
                y = a[1] + (b[1] - a[1]) * k / steps
                self.assertTrue(self.gmap.free(x, y, need),
                                'segment passes (%.2f, %.2f)' % (x, y))

    def test_plan_race_returns_a_drivable_path(self):
        res = plan_race(self.gmap, (0.6, 2.0), (3.4, 2.0))
        self.assertIsNotNone(res)
        pts = res['points']
        self.assertLess(res['length'], 4.0)
        self.assertGreater(res['time'], 1.0)
        self.assertLessEqual(float(np.max(res['speeds'])),
                             DEFAULT_CFG['v_max'] + 1e-6)
        for x, y in pts:
            self.assertGreater(self.gmap.clearance_at(x, y), 0.10,
                               'path point too close to a wall')
        self.assertLess(np.hypot(pts[-1][0] - 3.4, pts[-1][1] - 2.0), 0.25)

    def test_tight_goal_is_reached_anyway(self):
        # goal 0.03 m in front of a wall: no free cell there
        res = plan_race(self.gmap, (0.6, 2.0), (3.93, 2.0))
        self.assertIsNotNone(res)
        self.assertLess(np.hypot(res['points'][-1][0] - 3.93,
                                 res['points'][-1][1] - 2.0), 0.05)

    def test_speed_profile_respects_the_acceleration_limits(self):
        pts = resample([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], step=0.02)
        speeds, est = speed_profile(pts)
        self.assertEqual(len(speeds), len(pts))
        self.assertGreater(est, 0.0)
        self.assertLessEqual(float(np.max(speeds)), DEFAULT_CFG['v_max'] + 1e-6)
        # the two passes enforce the energy relation v2^2 = v1^2 + 2 a ds
        ds = np.linalg.norm(np.diff(np.asarray(pts, dtype=float), axis=0), axis=1)
        dv2 = np.diff(speeds ** 2)
        self.assertLessEqual(float(np.max(dv2 - 2.0 * DEFAULT_CFG['a_acc'] * ds)),
                             1e-6)
        self.assertGreaterEqual(
            float(np.min(dv2 + 2.0 * DEFAULT_CFG['a_dec'] * ds)), -1e-6)

    def test_speed_profile_respects_the_yaw_rate_limit(self):
        """A tight arc must be slowed until v * kappa fits the yaw rate cap."""
        radius = 0.08
        pts = [(radius * np.sin(a), radius - radius * np.cos(a))
               for a in np.linspace(0.0, np.pi / 2, 60)]
        pts = resample(pts, step=0.01)
        cfg = dict(DEFAULT_CFG, **{'max_yaw_rate': 1.40})
        speeds, _est = speed_profile(pts, cfg)
        kappa = effective_curvature(pts, cfg)
        self.assertLessEqual(float(np.max(speeds * kappa)),
                             cfg['max_yaw_rate'] + 1e-6)
        # without the cap the same arc would be driven much faster
        plain = dict(cfg, max_yaw_rate=1e9)
        fast, _ = speed_profile(pts, plain)
        self.assertGreater(float(np.max(fast * kappa)),
                           cfg['max_yaw_rate'] * 1.2)

    def test_smoothing_removes_the_45_degree_facets(self):
        """Chaikin turns the A* staircase into a C1 curve (no kink > 20 deg)."""
        path = [(1.00, 0.20), (1.20, 0.40), (1.35, 0.40), (1.55, 0.60)]
        out = smooth_path(resample(path, step=0.02))
        P = np.asarray(out, dtype=float)
        self.assertGreater(len(P), len(path))
        seg = np.diff(P, axis=0)
        ang = np.arctan2(seg[:, 1], seg[:, 0])
        jumps = np.abs(np.degrees(np.arctan2(np.sin(np.diff(ang)),
                                             np.cos(np.diff(ang)))))
        self.assertLess(float(np.max(jumps)), 20.0)

    def test_smoothed_corner_has_a_trackable_radius(self):
        """The planned curvature must stay inside what the tracker can steer."""
        gmap = GridMap.from_walls(door_room())
        cfg = dict(DEFAULT_CFG, **{'obstacle_radius': 0.18})
        raw = astar(gmap, (0.4, 0.4), (3.9, 2.0), cfg)
        path = resample(smoothing_pass(gmap, simplify(shortcut(gmap, raw, cfg),
                                                     tol=0.04), cfg), cfg=cfg)
        kappa = effective_curvature(path, cfg)
        speeds, _est = speed_profile(path, cfg)
        self.assertLessEqual(float(np.max(speeds * kappa)),
                             cfg['max_yaw_rate'] + 1e-6)
        self.assertLess(float(kappa.max()), 1.0 / 0.10)   # R > 10 cm everywhere

def leaky_room(cfg):
    """``door_room`` with no map frame and two holes in the south wall.

    The room keeps its inner door at y = 2, but the south wall is missing a
    piece near the start (x = 0.5) and another one near the goal (x = 3.5) --
    exactly the kind of gap a real explore pass leaves behind. Without a
    planning-domain limit A* hops out of the first hole, runs along the outside
    of the wall and comes back in through the second one: 3.4 m instead of the
    4.3 m indoor detour, and completely undrivable.
    """
    gmap = GridMap.from_walls(door_room(), cfg)
    occ = gmap.occupied.copy()
    occ[0, :] = occ[-1, :] = False       # drop the artificial map frame
    occ[:, 0] = occ[:, -1] = False
    for x in list(np.arange(0.20, 0.95, 0.05)) + list(np.arange(3.00, 3.75, 0.05)):
        i, j = gmap.index(x, -0.05)
        occ[j - 2:j + 3, i] = False               # two holes in the south wall
    return GridMap(occ, gmap.res, gmap.ox, gmap.oy)


class TestPlanningDomain(unittest.TestCase):
    """Keep the route inside the maze outline (two-pass follow up)."""

    cfg = {'bounds': (-0.9, 4.9, -0.9, 4.9), 'obstacle_radius': 0.15}
    start = (0.4, 0.4)
    goal = (3.4, 0.6)

    def setUp(self):
        self.gmap = leaky_room(self.cfg)

    def test_maze_bbox_is_the_wall_outline(self):
        i0, i1, j0, j1 = maze_bbox(self.gmap)
        x0, y0 = self.gmap.world(i0, j0)
        x1, y1 = self.gmap.world(i1, j1)
        self.assertAlmostEqual(x1 - x0, 4.2, delta=0.2)
        self.assertAlmostEqual(y1 - y0, 4.2, delta=0.2)

    def test_restriction_blocks_the_margin_but_keeps_the_goal_pocket(self):
        gmap, info = restrict_to_maze(self.gmap, self.goal, self.cfg)
        self.assertGreater(info['blocked'], 0)
        self.assertAlmostEqual(info['bbox'][0], 0.0, delta=0.15)
        # a cell well outside the room is blocked now ...
        self.assertTrue(gmap.occupied[gmap.index(-0.6, 2.0)[1],
                                      gmap.index(-0.6, 2.0)[0]])
        # ... but the goal itself is still free (pocket around it)
        self.assertTrue(gmap.free(self.goal[0], self.goal[1], 0.06))

    def test_unrestricted_plan_leaves_the_room_through_the_holes(self):
        res = plan_race(self.gmap, self.start, self.goal,
                        dict(self.cfg, restrict_to_maze=0.0))
        self.assertIsNotNone(res)
        _on_wall, outside = check_route(self.gmap, res['points'], self.goal,
                                        self.cfg)
        self.assertGreater(outside, 0)

    def test_restricted_plan_stays_inside_the_outline(self):
        res = plan_race(self.gmap, self.start, self.goal,
                        dict(self.cfg, restrict_to_maze=1.0))
        self.assertIsNotNone(res)
        on_wall, outside = check_route(self.gmap, res['points'], self.goal,
                                       self.cfg)
        self.assertEqual((on_wall, outside), (0, 0))
        self.assertLess(np.hypot(res['points'][-1][0] - self.goal[0],
                                 res['points'][-1][1] - self.goal[1]), 0.20)

    def test_map_info_is_reported(self):
        res = plan_race(self.gmap, self.start, self.goal, self.cfg)
        self.assertIn('map_info', res)
        self.assertGreater(res['map_info']['blocked'], 0)
        self.assertIsNotNone(res['map_info']['bbox'])

    def test_check_route_flags_points_outside_the_outline(self):
        pts = [(0.4, 0.4), (-0.6, 2.0), (3.6, 3.5)]
        _on_wall, outside = check_route(self.gmap, pts, self.goal, self.cfg)
        self.assertEqual(outside, 1)


if __name__ == '__main__':
    unittest.main()
