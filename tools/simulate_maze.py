#!/usr/bin/env python3
"""Headless 2D simulator for the maze left-wall follower.

Why this exists
---------------
Tuning the follower directly in Gazebo costs ~30 s per launch and the physics
adds noise (wheel slip, contacts).  This tool keeps the *exact same* control
code (``maze_control.control_law.WallFollowerController`` and
``maze_control.utils.scan_sectors``) but replaces Gazebo with:

* a unicycle kinematic model,
* a ray caster against the very same wall boxes that are written into the
  ``.world`` file (``generate_maze.wall_segments``).

So a parameter set that solves the maze here solves it in Gazebo too, and a
tuning loop costs a second instead of a minute.

Usage
-----
    python3 tools/simulate_maze.py                # default parameters
    python3 tools/simulate_maze.py --front_stop 0.5 --open_distance 0.65
    python3 tools/simulate_maze.py --trace --plot
"""

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WORKSPACE, 'maze_ws', 'src', 'maze_control'))

from generate_maze import (COLS, ROWS, SEED, Maze,  # noqa: E402
                           wall_segments)
from maze_control.control_law import (DEFAULT_WALL_CFG,  # noqa: E402
                                      WallFollowerController)
from maze_control.utils import normalize_angle, scan_sectors  # noqa: E402

LIDAR_RANGE_MAX = 3.5
LIDAR_RAYS = 360


class FakeScan:
    """Duck-typed stand-in for sensor_msgs/LaserScan (only 3 fields used)."""

    __slots__ = ('angle_min', 'angle_increment', 'ranges')

    def __init__(self, angle_min, angle_increment, ranges):
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.ranges = ranges


def ray_cast(x, y, yaw, boxes, n=LIDAR_RAYS):
    """Distance to the nearest wall for ``n`` rays spread over 360 degrees.

    Angles are in the laser frame: index 0 -> -pi and the angle increases
    counter-clockwise (left is positive), matching the sampling of the Gazebo
    ray sensor configured in the URDF.  Returns a duck-typed ``FakeScan``.
    """
    local = -math.pi + (2.0 * math.pi) * np.arange(n) / n
    world = yaw + local
    dx = np.cos(world)
    dy = np.sin(world)
    best = np.full(n, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for (cx, cy, sx, sy) in boxes:
            hx = 0.5 * sx
            hy = 0.5 * sy
            tx1 = (cx - hx - x) / dx
            tx2 = (cx + hx - x) / dx
            ty1 = (cy - hy - y) / dy
            ty2 = (cy + hy - y) / dy
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = (tmax >= tmin) & (tmax >= 0.0) & (tmin > 0.0)
            best = np.where(hit & (tmin < best), tmin, best)
    best = np.where(np.isfinite(best), best, math.inf)
    ranges = np.minimum(best, LIDAR_RANGE_MAX)
    return FakeScan(-math.pi, 2.0 * math.pi / n, [float(r) for r in ranges])


def simulate(cfg, start=(0.40, 0.40, 0.0), dt=0.05, timeout=600.0,
             trace=False, plot=False, goal=(3.0, 4.10)):
    maze = Maze(COLS, ROWS, SEED)
    boxes = [(cx, cy, sx, sy) for (cx, cy, sx, sy, _name) in wall_segments(maze)]

    ctrl = WallFollowerController(cfg)
    x, y, yaw = start
    t = 0.0
    path = [(x, y)]
    collisions = 0
    min_clearance = math.inf
    last_trace = -1.0
    result = 'timeout'
    state_time = {}

    while t < timeout:
        scan = ray_cast(x, y, yaw, boxes)
        s = scan_sectors(scan)
        min_clearance = min(min_clearance, s['front'], s['left'], s['right'])
        if min(scan.ranges) < 0.09:
            collisions += 1
            result = 'collision at (%.2f, %.2f) t=%.1f' % (x, y, t)
            break

        v, w, state = ctrl.step(s['front_center'], s['left'], s['right'],
                                yaw, dt, pose=(x, y), front_wide=s['front'])
        state_time[state] = state_time.get(state, 0.0) + dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + w * dt)
        t += dt
        path.append((x, y))

        if trace and t - last_trace >= 1.0:
            last_trace = t
            print('t=%5.1f pos=(%.2f,%.2f) yaw=%+6.1f %-6s '
                  'front=%.2f left=%.2f right=%.2f cmd=(%.2f,%+.2f)'
                  % (t, x, y, math.degrees(yaw), state, s['front'], s['left'],
                     s['right'], v, w))
        if ctrl.last_event == 'turn_extend':
            print('  turn %2d  t=%5.1f pos=(%.2f,%.2f)  extend           '
                  'front=%.2f -> target %+6.1f'
                  % (ctrl.turns, t, x, y, s['front'],
                     math.degrees(ctrl.target_yaw)))
        if ctrl.last_event == 'turn_start':
            print('  turn %2d  t=%5.1f pos=(%.2f,%.2f) yaw=%+6.1f  %-18s '
                  'front=%.2f left=%.2f right=%.2f -> target %+6.1f'
                  % (ctrl.turns, t, x, y, math.degrees(yaw),
                     ctrl.last_reason, s['front'], s['left'], s['right'],
                     math.degrees(ctrl.target_yaw)))

        if y >= goal[1] and x >= goal[0]:
            result = 'success'
            break
        if not (-0.2 <= x <= 4.2 and -0.2 <= y <= 4.6):
            result = 'left the maze at (%.2f, %.2f)' % (x, y)
            break

    print('\nresult: %s' % result)
    print('time: %.1f s   path length: %.2f m   turns: %d   min clearance: %.2f m'
          % (t, path_length(path), ctrl.turns, min_clearance))
    if state_time:
        print('state time: ' + '  '.join('%s=%.1fs' % kv for kv in
                                         sorted(state_time.items())))
    if collisions:
        print('collisions:', collisions)
    if plot:
        ascii_path(path)
    return {
        'ok': result == 'success',
        'result': result,
        'time': t,
        'turns': ctrl.turns,
        'path': path,
        'min_clearance': min_clearance,
        'collisions': collisions,
        'state_time': state_time,
    }


def path_length(path):
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def ascii_path(path, step=0.05):
    w, h = 100, 50
    grid = [[' '] * w for _ in range(h)]

    def to_cell(px, py):
        cx = int(px / 4.2 * (w - 1))
        cy = int((1.0 - py / 4.6) * (h - 1))
        return max(0, min(w - 1, cx)), max(0, min(h - 1, cy))

    for (px, py) in path[::40]:
        cx, cy = to_cell(px, py)
        grid[cy][cx] = '.'
    sx, sy = to_cell(*path[0])
    grid[sy][sx] = 'S'
    ex, ey = to_cell(*path[-1])
    grid[ey][ex] = 'E'
    print('\n' + '\n'.join(''.join(row) for row in grid))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trace', action='store_true', help='print a trace every second')
    ap.add_argument('--plot', action='store_true', help='ASCII sketch of the driven path')
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--start', type=float, nargs=3, default=[0.40, 0.40, 0.0],
                    metavar=('X', 'Y', 'YAW'))
    for key, value in DEFAULT_WALL_CFG.items():
        ap.add_argument('--' + key, type=float, default=value)
    args = ap.parse_args()

    cfg = {}
    for key in DEFAULT_WALL_CFG:
        cfg[key] = int(getattr(args, key)) if key == 'blocked_ticks' else float(getattr(args, key))

    run = simulate(cfg, start=tuple(args.start), dt=0.05,
                   timeout=args.timeout, trace=args.trace,
                   plot=args.plot)
    sys.exit(0 if run['ok'] else 1)


if __name__ == '__main__':
    main()
