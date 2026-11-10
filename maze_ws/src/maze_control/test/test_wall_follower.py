"""Unit tests for the maze wall-following state machine."""

import math
import unittest

from maze_control.control_law import WallFollowerController


def run(ctrl, readings, yaw, ticks):
    """Feed the same sensor reading for ``ticks`` steps, return last result."""
    out = None
    for _ in range(ticks):
        out = ctrl.step(readings[0], readings[1], readings[2], yaw)
        v, w, state = out
        yaw += w * 0.05
        out = (v, w, state, yaw)
    return out


def start_dead_end_turn(ctrl, limit=30):
    """Feed a blocked-everywhere picture until the U-turn is latched.

    The follower holds the dead-end verdict for ``deadend_ticks`` ticks before
    it spins 180 deg, so four ticks are no longer enough.
    """
    for _ in range(limit):
        ctrl.step(0.30, 0.30, 0.30, 0.0)
        if ctrl.last_event == 'turn_start':
            return True
    return False


def run_fixed(ctrl, front, left, right, yaw=0.0, ticks=80):
    """Hold the same sector readings for ``ticks`` steps and return the last command."""
    v = w = 0.0
    state = 'FOLLOW'
    for _ in range(ticks):
        v, w, state = ctrl.step(front, left, right, yaw)
    return v, w, state


class TestWallFollower(unittest.TestCase):

    def setUp(self):
        self.ctrl = WallFollowerController()

    def test_corridor_keeps_left_distance(self):
        # 0.8 m wide corridor, robot slightly off the target distance
        v, w, state = self.ctrl.step(1.5, 0.50, 0.30, 0.0)
        self.assertEqual(state, 'FOLLOW')
        self.assertGreater(v, 0.0)
        # too far from the left wall -> steer left (positive yaw rate)
        self.assertGreater(w, 0.0)

    def test_corridor_too_close_steers_right(self):
        _, w, state = self.ctrl.step(1.5, 0.20, 0.60, 0.0)
        self.assertEqual(state, 'FOLLOW')
        self.assertLess(w, 0.0)

    def test_blocked_with_left_opening_turns_left(self):
        for _ in range(3):
            self.ctrl.step(0.30, 1.00, 0.30, 0.0)
        v, w, state = self.ctrl.step(0.30, 1.00, 0.30, 0.0)
        self.assertEqual(state, 'TURN')
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)          # rotate counter-clockwise

    def test_dead_end_turns_around(self):
        self.assertTrue(start_dead_end_turn(self.ctrl))
        self.assertEqual(self.ctrl.state, 'TURN')
        self.assertEqual(self.ctrl.last_reason, 'dead end')
        # the latched goal is 180 degrees away from the current heading
        err = abs((self.ctrl.target_yaw - 0.0 + math.pi) % (2 * math.pi) - math.pi)
        self.assertAlmostEqual(err, math.pi, places=3)

    def test_turn_finishes_and_returns_to_follow(self):
        # start turning, then converge to the latched heading
        for _ in range(4):
            self.ctrl.step(0.30, 1.00, 0.30, 0.0)
        target = self.ctrl.target_yaw
        for _ in range(200):
            v, w, state = self.ctrl.step(1.20, 0.40, 0.60, target)
            if state == 'FOLLOW':
                break
        self.assertEqual(state, 'FOLLOW')
        self.assertEqual(self.ctrl.last_event, 'turn_done')

    def test_speed_rises_with_clearance(self):
        # a long clear corridor allows the cruise speed ...
        v_open, _, _ = run_fixed(WallFollowerController(), 10.0, 0.34, 0.50)
        # ... while a wall 0.55 m ahead keeps the robot slow
        v_tight, _, _ = run_fixed(WallFollowerController(), 0.55, 0.34, 0.50)
        self.assertGreater(v_open, v_tight + 0.02)
        self.assertAlmostEqual(v_open, WallFollowerController().cfg['speed'],
                               places=2)

    def test_speed_is_acceleration_limited_from_rest(self):
        ctrl = WallFollowerController()
        v, _, _ = ctrl.step(10.0, 0.34, 0.50, 0.0)
        self.assertLessEqual(v, ctrl.cfg['accel'] * 0.05 + 1e-9)
        self.assertGreater(v, 0.0)

    def test_speed_never_exceeds_cruise(self):
        ctrl = WallFollowerController()
        v, _, _ = run_fixed(ctrl, 10.0, 0.34, 0.50, ticks=200)
        self.assertLessEqual(v, ctrl.cfg['speed'] + 1e-9)

    def test_hard_steering_slows_the_robot_down(self):
        aligned = WallFollowerController()
        v_aligned, w_aligned, _ = run_fixed(aligned, 1.5, 0.34, 0.50, yaw=0.0)
        turning = WallFollowerController()
        turning.step(1.5, 0.34, 0.50, 0.0)          # latch the heading
        v_turning, w_turning, _ = run_fixed(turning, 1.5, 0.34, 0.50, yaw=0.6)
        self.assertLess(abs(w_aligned), 0.1)
        self.assertGreater(abs(w_turning), 0.5)
        self.assertLess(v_turning, v_aligned - 0.02)

    def test_not_blocked_again_right_after_turn(self):
        for _ in range(4):
            self.ctrl.step(0.30, 1.00, 0.30, 0.0)
        target = self.ctrl.target_yaw
        for _ in range(200):
            _, _, state = self.ctrl.step(1.20, 0.40, 0.60, target)
            if state == 'FOLLOW':
                break
        # a single blocked tick must not immediately re-trigger a turn
        _, _, state = self.ctrl.step(0.30, 1.00, 0.30, target)
        self.assertEqual(state, 'FOLLOW')

    # --- corner cutting ----------------------------------------------------
    def test_corner_disabled_uses_pivot_turn(self):
        ctrl = WallFollowerController({'corner_enabled': 0.0})
        for _ in range(4):
            _, _, state = ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        self.assertEqual(state, 'TURN')
        self.assertIsNone(ctrl.corner_path)

    def test_corner_path_cuts_through_the_doorway(self):
        ctrl = WallFollowerController()
        for _ in range(4):
            _, _, state = ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        self.assertEqual(state, 'CORNER')
        self.assertGreaterEqual(len(ctrl.corner_path), 3)
        end_x, end_y = ctrl.corner_path[-1]
        # a left turn from an eastbound corridor exits into the north corridor
        self.assertGreater(end_y - 0.41, 0.5)
        self.assertAlmostEqual(end_x, 1.16, delta=0.35)

    def test_corner_aborts_when_clearance_is_unsafe(self):
        ctrl = WallFollowerController()
        for _ in range(4):
            _, _, state = ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        self.assertEqual(state, 'CORNER')
        # the abort is debounced: at 5 Hz one stale scan must not end the arc,
        # so the state only changes after corner_abort_ticks unsafe ticks
        ctrl.corner_time = ctrl.cfg['corner_abort_grace']
        self.assertEqual(ctrl.step(0.30, 0.05, 0.30, 0.0, pose=(1.16, 0.41))[2],
                         'CORNER')
        for _ in range(ctrl.cfg['corner_abort_ticks'] - 1):
            _, _, state = ctrl.step(0.30, 0.05, 0.30, 0.0, pose=(1.16, 0.41))
        self.assertEqual(state, 'TURN')

    def test_stuck_robot_backs_out(self):
        ctrl = WallFollowerController()
        # commanded forward, but the pose never changes -> recovery kicks in
        for _ in range(40):
            v, w, state = ctrl.step(1.00, 0.40, 0.40, 0.0, pose=(1.00, 1.00))
            if state == 'RECOVER':
                break
        self.assertEqual(state, 'RECOVER')
        self.assertLess(v, 0.0)                       # reversing
        for _ in range(40):
            v, w, state = ctrl.step(1.00, 0.40, 0.40, 0.0, pose=(1.00, 1.00))
            if state == 'FOLLOW':
                break
        self.assertEqual(state, 'FOLLOW')

    def test_z_bend_merges_into_one_s(self):
        ctrl = WallFollowerController()
        # corner 1: left opening -> left turn, path driven to the end
        for _ in range(4):
            ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        self.assertEqual(ctrl.state, 'CORNER')
        for _ in range(400):
            pose = ctrl.corner_path[-1]
            _, _, state = ctrl.step(0.60, 0.40, 0.60, ctrl.target_yaw, pose=pose)
            if state == 'FOLLOW':
                break
        self.assertEqual(ctrl.state, 'FOLLOW')
        # corner 2, 0.3 m later and turning the other way -> chained S
        for _ in range(20):
            _, _, state = ctrl.step(0.30, 0.40, 0.60, ctrl.target_yaw,
                                    pose=(pose[0], pose[1] + 0.30))
            if state == 'CORNER':
                break
        self.assertEqual(state, 'CORNER')
        self.assertTrue(ctrl.chained)
        self.assertLess(ctrl.turn_delta * ctrl.prev_corner_delta, 0.0)

    def test_dead_end_turn_extends_while_front_is_blocked(self):
        # start a dead-end turn (latched 180 deg)
        self.assertTrue(start_dead_end_turn(self.ctrl))
        self.assertEqual(self.ctrl.state, 'TURN')
        target = self.ctrl.target_yaw
        turns = self.ctrl.turns
        # arrive at the latched heading, but the wall is still 0.30 m ahead:
        # the turn must not end (that is what used to oscillate +-90 deg)
        v, w, state = self.ctrl.step(0.30, 0.40, 0.30, target)
        self.assertEqual(state, 'TURN')
        self.assertEqual(self.ctrl.last_event, 'turn_extend')
        self.assertEqual(v, 0.0)
        self.assertEqual(self.ctrl.turns, turns)       # same turn episode
        err = abs((self.ctrl.target_yaw - target + math.pi) % (2 * math.pi) - math.pi)
        self.assertAlmostEqual(err, math.pi / 2, places=3)

    def test_dead_end_turn_gives_up_after_one_circle(self):
        self.assertTrue(start_dead_end_turn(self.ctrl))
        # rotate all the way round with a blocked front the whole time
        state = 'TURN'
        for _ in range(12):
            v, w, state = self.ctrl.step(0.30, 0.40, 0.30, self.ctrl.target_yaw)
            if state == 'FOLLOW':
                break
        self.assertEqual(state, 'FOLLOW')
        self.assertEqual(self.ctrl.last_event, 'turn_done')
        self.assertLessEqual(self.ctrl.turn_total, 2 * math.pi + 1e-6)

    def test_open_turn_completes_even_with_a_close_front(self):
        # a normal corner turn (not a dead end) must finish at once: a wall
        # 0.30 m after the turn is a corridor, not a reason to keep rotating
        for _ in range(4):
            self.ctrl.step(0.30, 1.00, 0.30, 0.0)
        target = self.ctrl.target_yaw
        for _ in range(200):
            v, w, state = self.ctrl.step(0.30, 0.40, 0.60, target)
            if state != 'TURN':
                break
        self.assertEqual(state, 'FOLLOW')
        self.assertEqual(self.ctrl.last_event, 'turn_done')

    def test_pivot_loop_in_a_narrow_slot_backs_out(self):
        # A robot wedged in a slot keeps pivoting between two dead ends: three
        # pivots inside 0.2 m must turn into a backup instead of a 4th pivot.
        # (stuck detection is off here so that only the loop rule can fire)
        ctrl = WallFollowerController({'stuck_enabled': 0.0})
        pose = (1.00, 1.00)
        yaw = 0.0
        for _ in range(1200):
            v, w, state = ctrl.step(0.30, 0.12, 0.55, yaw, 0.05, pose)
            yaw += w * 0.05
            if state == 'RECOVER':
                break
        self.assertEqual(state, 'RECOVER')
        self.assertEqual(ctrl.last_reason, 'turn loop')
        self.assertLess(v, 0.0)             # backing out
        self.assertLessEqual(ctrl.turns, 3)

    def test_pivots_after_real_motion_are_not_a_loop(self):
        # the same pivot repeated, but the robot moved 0.4 m in between, is an
        # ordinary corner sequence and must not trigger a backup
        self.ctrl.step(0.30, 0.12, 0.55, 0.0, 0.05, (1.00, 1.00))
        self.ctrl.step(0.30, 0.12, 0.55, 0.0, 0.05, (1.00, 1.00))
        self.ctrl.step(0.30, 0.12, 0.55, 0.0, 0.05, (1.00, 1.40))
        self.ctrl.step(0.30, 0.12, 0.55, 0.0, 0.05, (1.00, 1.80))
        self.ctrl.state = 'FOLLOW'
        self.ctrl.cooldown = 0.0
        for _ in range(4):
            v, w, state = self.ctrl.step(0.30, 0.12, 0.55, 0.0, 0.05, (1.00, 2.20))
        self.assertNotEqual(state, 'RECOVER')

    def test_chain_can_be_disabled(self):
        ctrl = WallFollowerController({'corner_chain_enabled': 0.0})
        for _ in range(4):
            ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        pose = ctrl.corner_path[-1]
        for _ in range(400):
            _, _, state = ctrl.step(0.60, 0.40, 0.60, ctrl.target_yaw, pose=pose)
            if state == 'FOLLOW':
                break
        for _ in range(20):
            _, _, state = ctrl.step(0.30, 0.40, 0.60, ctrl.target_yaw,
                                    pose=(pose[0], pose[1] + 0.30))
            if state == 'CORNER':
                break
        self.assertEqual(state, 'CORNER')
        self.assertFalse(ctrl.chained)

    @staticmethod
    def _max_chord_deviation(pts):
        x0, y0 = pts[0]
        x1, y1 = pts[-1]
        den = (x1 - x0) ** 2 + (y1 - y0) ** 2
        dev = 0.0
        for px, py in pts:
            t = ((px - x0) * (x1 - x0) + (py - y0) * (y1 - y0)) / den
            dev = max(dev, math.hypot(px - (x0 + (x1 - x0) * t),
                                      py - (y0 + (y1 - y0) * t)))
        return dev

    def test_diagonal_path_is_a_straight_chord(self):
        def path_of(cfg):
            ctrl = WallFollowerController(cfg)
            for _ in range(4):
                ctrl.step(0.40, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
            return ctrl.corner_path

        diagonal = path_of({'corner_diagonal': 1.0})
        rounded = path_of({'corner_diagonal': 0.0})
        self.assertIsNotNone(diagonal)
        self.assertIsNotNone(rounded)
        self.assertLess(self._max_chord_deviation(diagonal), 1e-6)
        self.assertGreater(self._max_chord_deviation(rounded), 0.01)

    def test_tight_chord_small_arc(self):
        # a huge required clearance forces the "small arc adjustment" waypoint
        ctrl = WallFollowerController({'corner_diagonal': 1.0,
                                       'corner_min_clearance': 0.60})
        for _ in range(4):
            ctrl.step(0.30, 1.00, 0.30, 0.0, pose=(1.16, 0.41))
        path = ctrl.corner_path
        self.assertIsNotNone(path)
        # the path must bow away from the straight chord to the exit point
        self.assertGreater(self._max_chord_deviation(path), 0.02)

    def test_corner_needs_pose(self):
        ctrl = WallFollowerController()
        for _ in range(4):
            _, _, state = ctrl.step(0.30, 1.00, 0.30, 0.0)
        self.assertEqual(state, 'TURN')


if __name__ == '__main__':
    unittest.main()


class NarrowFrontTest(unittest.TestCase):
    """The narrow cone decides "blocked", the wide fan only slows the robot."""

    def test_wall_hugging_pose_does_not_start_a_turn(self):
        ctrl = WallFollowerController()
        # 0.36 m in the wide fan (the corridor's own side wall) but 1.15 m
        # dead ahead: no turn may start.
        for _ in range(20):
            v, w, state = ctrl.step(1.15, 0.17, 0.53, 0.0, 0.05,
                                    pose=(1.38, 1.20), front_wide=0.36)
        self.assertEqual(state, 'FOLLOW')
        self.assertNotEqual(ctrl.last_event, 'turn_start')

    def test_wide_front_still_holds_the_old_behaviour(self):
        ctrl = WallFollowerController()
        # without the narrow cone the same geometry is read as a dead end
        for _ in range(20):
            v, w, state = ctrl.step(0.36, 0.17, 0.53, 0.0, 0.05,
                                    pose=(1.38, 1.20))
        self.assertEqual(state, 'TURN')


class DeadEndConfirmationTest(unittest.TestCase):

    def test_u_turn_waits_for_the_verdict_to_hold(self):
        ctrl = WallFollowerController()
        readings = (0.30, 0.20, 0.30)
        started = None
        for tick in range(1, 21):
            ctrl.step(readings[0], readings[1], readings[2], 0.0, 0.05,
                      pose=(1.38, 1.20))
            if ctrl.last_event == 'turn_start':
                started = tick
                break
        self.assertIsNotNone(started)
        # blocked_ticks = 3, then deadend_ticks = 8 more ticks
        self.assertGreaterEqual(started, 10)

    def test_dead_end_hold_keeps_the_robot_still(self):
        ctrl = WallFollowerController()
        for _ in range(6):
            v, w, state = ctrl.step(0.30, 0.20, 0.30, 0.0, 0.05,
                                    pose=(1.38, 1.20))
        self.assertEqual(ctrl.last_event, 'deadend_hold')
        self.assertAlmostEqual(v, 0.0)
