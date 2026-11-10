"""Unit tests for the pure control laws (no ROS / no Gazebo needed)."""

import math

from maze_control.control_law import point_command, wall_follow_command
from types import SimpleNamespace

from maze_control.utils import clamp, normalize_angle, scan_sectors

CFG = {
    'front_stop': 0.45,
    'turn_clearance': 0.42,
    'turn_speed': 1.1,
    'target_left': 0.30,
    'kp': 3.0,
    'max_w': 1.0,
    'speed': 0.16,
    'slow_distance': 0.70,
    'slow_speed': 0.07,
}

PCFG = {
    'tolerance': 0.05,
    'rotate_threshold': 0.5,
    'kp_linear': 0.8,
    'max_linear': 0.2,
    'kp_angular': 1.6,
    'max_angular': 1.0,
}


def test_clamp_and_normalize():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert abs(normalize_angle(3 * math.pi) - math.pi) < 1e-9
    assert abs(normalize_angle(-3 * math.pi) + math.pi) < 1e-9


def test_wall_follow_straight_corridor():
    v, w = wall_follow_command(front=3.0, left=0.30, right=0.40, cfg=CFG)
    assert v == CFG['speed']
    assert abs(w) < 1e-9


def test_wall_follow_too_far_from_left_wall_turns_left():
    v, w = wall_follow_command(front=3.0, left=0.60, right=0.10, cfg=CFG)
    assert v == CFG['speed']
    assert w > 0.0


def test_wall_follow_front_wall_turns_left_when_open():
    v, w = wall_follow_command(front=0.30, left=0.80, right=0.35, cfg=CFG)
    assert v == 0.0
    assert w > 0.0


def test_wall_follow_dead_end_turns_right():
    v, w = wall_follow_command(front=0.30, left=0.30, right=0.30, cfg=CFG)
    assert v == 0.0
    assert w < 0.0


def test_wall_follow_slow_down_near_front_wall():
    v, _ = wall_follow_command(front=0.60, left=0.30, right=0.40, cfg=CFG)
    assert abs(v - CFG['slow_speed']) < 1e-9


def test_point_command_rotates_in_place_when_behind():
    v, w, distance, heading = point_command(0.0, 0.0, 0.0, -1.0, 0.0, PCFG)
    assert v == 0.0
    assert abs(abs(heading) - math.pi) < 1e-9
    assert abs(abs(w) - PCFG['max_angular']) < 1e-9


def test_point_command_drives_forward_when_aligned():
    v, w, distance, heading = point_command(0.0, 0.0, 0.0, 0.5, 0.0, PCFG)
    assert v > 0.0
    assert abs(w) < 1e-9
    assert abs(heading) < 1e-9
    assert abs(distance - 0.5) < 1e-9


def test_point_command_stops_inside_tolerance():
    v, w, distance, _ = point_command(0.0, 0.0, 0.0, 0.02, 0.0, PCFG)
    assert v == 0.0 and w == 0.0
    assert distance < PCFG['tolerance']


def _side_wall_scan():
    """Scan whose only nearby return comes from a wall 20 deg off the heading.

    This is the pose that made seed 3 fail: the robot hugs one wall in a 0.7 m
    corridor, so the wall sits inside the wide +-20 deg fan even though the
    corridor straight ahead is clear.
    """
    n = 360
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        ranges.append(0.36 if 0.30 <= a <= 0.40 else 3.5)
    return SimpleNamespace(angle_min=-math.pi, angle_increment=inc,
                           ranges=ranges)


def test_side_wall_is_only_in_the_wide_front_sector():
    s = scan_sectors(_side_wall_scan())
    assert s['front'] < 0.45          # the wide fan does see that wall ...
    assert s['front_center'] > 1.0    # ... but the +-6 deg cone does not
