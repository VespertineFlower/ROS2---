"""Pure-Python control laws (no ROS imports).

Keeping the laws here makes them reusable by the ROS nodes and by the
headless validation simulator (tools/simulate_maze.py).
"""

import math

from .utils import clamp, normalize_angle


def wall_follow_command(front, left, right, cfg):
    """Left-wall following law (simple reference model).

    This is the textbook reactive law that the project started from.  The
    production node uses :class:`WallFollowerController` instead; this
    function is kept as a readable reference and is covered by unit tests.

    Parameters
    ----------
    front, left, right : float
        Sector distances in metres.
    cfg : dict
        front_stop, turn_clearance, turn_speed, target_left, kp, max_w,
        speed, slow_distance, slow_speed.

    Returns
    -------
    (v, w) : tuple[float, float]
        Linear and angular velocity commands.
    """
    if front < cfg['front_stop']:
        # A wall is directly ahead.  The left-hand rule says "turn left",
        # but only if the left side is actually free; otherwise turn right
        # (this also produces the 180 deg U-turn at a dead end).
        if left > cfg['turn_clearance'] and left >= right:
            return 0.0, cfg['turn_speed']
        return 0.0, -cfg['turn_speed']

    err = left - cfg['target_left']
    if abs(err) < 0.015:
        w = 0.0
    else:
        w = clamp(cfg['kp'] * err, -cfg['max_w'], cfg['max_w'])

    v = cfg['speed']
    if front < cfg['slow_distance']:
        v = cfg['slow_speed']
    return v, w


def point_command(x, y, yaw, goal_x, goal_y, cfg):
    """P controller for "go to one point" (with a heading-first phase).

    Returns (v, w, distance, heading_error).
    """
    dx = goal_x - x
    dy = goal_y - y
    distance = math.hypot(dx, dy)
    heading_error = 0.0
    if distance > 1e-6:
        desired = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(desired - yaw),
                                   math.cos(desired - yaw))

    if distance <= cfg['tolerance']:
        return 0.0, 0.0, distance, heading_error

    if abs(heading_error) > cfg['rotate_threshold']:
        # Turn in place until roughly aligned, then drive.
        w = clamp(cfg['kp_angular'] * heading_error,
                  -cfg['max_angular'], cfg['max_angular'])
        return 0.0, w, distance, heading_error

    v = clamp(cfg['kp_linear'] * distance,
              -cfg['max_linear'], cfg['max_linear'])
    # do not drive fast while the heading is still off
    v *= max(0.0, math.cos(heading_error))
    w = clamp(cfg['kp_angular'] * heading_error,
              -cfg['max_angular'], cfg['max_angular'])
    return v, w, distance, heading_error


# ---------------------------------------------------------------------------
# Pure pursuit path tracking
# ---------------------------------------------------------------------------

DEFAULT_PP_CFG = {
    'lookahead': 0.32,        # m, distance of the carrot point
    'min_lookahead': 0.18,    # m, clamp for low speed
    'max_lookahead': 0.55,    # m
    'lookahead_gain': 0.8,    # L = clamp(gain * v + lookahead, min, max)
    'max_linear': 0.20,       # m/s
    'min_linear': 0.05,       # m/s
    'max_angular': 1.4,       # rad/s
    'goal_tolerance': 0.06,   # m
    'slow_angle': 0.9,        # rad, |alpha| at which the speed is scaled down
    'slow_factor': 0.45,      # speed factor at large heading error
    'corner_radius': 0.18,    # m, path corner rounding
    'corner_samples': 6,      # points per rounded corner
    'end_slowdown': 0.45,     # m, distance to the goal where we slow down
}


def path_length(path):
    """Total length of a polyline given as [(x, y), ...]."""
    total = 0.0
    for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
        total += math.hypot(bx - ax, by - ay)
    return total


def smooth_path(points, radius=0.18, samples=6):
    """Round the corners of a polyline with quadratic Bezier arcs.

    A pure pursuit controller cuts corners, so feeding it a path with sharp
    90 deg vertices makes it swing wide and clip walls.  Rounding the corners
    keeps the reference path inside the corridor.
    """
    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) < 3 or radius <= 0.0:
        return pts

    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        d1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        d2 = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if d1 < 1e-6 or d2 < 1e-6:
            continue
        r = min(radius, 0.5 * d1, 0.5 * d2)
        ax, ay = p1[0] + (p0[0] - p1[0]) / d1 * r, p1[1] + (p0[1] - p1[1]) / d1 * r
        bx, by = p1[0] + (p2[0] - p1[0]) / d2 * r, p1[1] + (p2[1] - p1[1]) / d2 * r
        for k in range(1, samples + 1):
            t = k / float(samples)
            u = 1.0 - t
            x = u * u * ax + 2.0 * u * t * p1[0] + t * t * bx
            y = u * u * ay + 2.0 * u * t * p1[1] + t * t * by
            if math.hypot(x - out[-1][0], y - out[-1][1]) > 1e-4:
                out.append((x, y))
    if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) > 1e-4:
        out.append(pts[-1])
    return out


def _project_on_path(x, y, path, start_index):
    """Closest point on the polyline (searching from ``start_index``)."""
    best = (float('inf'), start_index, path[start_index][0], path[start_index][1],
            start_index)
    n = len(path)
    for i in range(max(0, start_index), n - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            t = 0.0
        else:
            t = ((x - ax) * dx + (y - ay) * dy) / seg2
            t = clamp(t, 0.0, 1.0)
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - x, py - y)
        if d < best[0]:
            best = (d, i, px, py, t)
    return best


def _line_path(x0, y0, x1, y1, samples):
    """Straight polyline from (x0, y0) to (x1, y1) with ``samples`` points."""
    n = max(2, samples)
    return [(x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * i / (n - 1))
            for i in range(n)]


def _project_param(px, py, x0, y0, x1, y1):
    """Parameter t in [0, 1] of the projection of P on segment A->B."""
    vx, vy = x1 - x0, y1 - y0
    den = vx * vx + vy * vy
    if den <= 1e-12:
        return 0.0
    return clamp(((px - x0) * vx + (py - y0) * vy) / den, 0.0, 1.0)


def _point_segment_distance(px, py, x0, y0, x1, y1):
    t = _project_param(px, py, x0, y0, x1, y1)
    return math.hypot(px - (x0 + (x1 - x0) * t), py - (y0 + (y1 - y0) * t))


def _carrot_point(path, index, t, lookahead):
    """Walk ``lookahead`` metres along the path from a projection point."""
    n = len(path)
    px, py = path[index][0], path[index][1]
    bx, by = path[index + 1]
    seg_len = math.hypot(bx - px, by - py)
    travelled = (1.0 - t) * seg_len
    if travelled >= lookahead:
        ratio = lookahead / seg_len if seg_len > 1e-9 else 0.0
        return (px + (bx - px) * t + (bx - px) * ratio,
                py + (by - py) * t + (by - py) * ratio, None)
    for j in range(index + 1, n - 1):
        ax, ay = path[j]
        bx, by = path[j + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        travelled += seg_len
        if travelled >= lookahead:
            ratio = (seg_len - (travelled - lookahead)) / seg_len if seg_len > 1e-9 else 0.0
            ratio = clamp(ratio, 0.0, 1.0)
            return (ax + (bx - ax) * ratio, ay + (by - ay) * ratio,
                    j if j > index else None)
    return (path[-1][0], path[-1][1], None)


def pure_pursuit_command(x, y, yaw, path, cfg, index_hint=0):
    """Classic pure pursuit.

    The carrot point sits ``lookahead`` metres ahead on the path; the
    curvature that reaches it follows from the geometry of a circular arc:
    ``kappa = 2 * sin(alpha) / L`` with ``alpha`` the angle between the
    heading and the carrot point.  ``omega = v * kappa``.

    Returns ``(v, w, carrot, index, remaining, alpha)`` where ``remaining``
    is the path length still to travel and ``alpha`` the relative bearing.
    """
    n = len(path)
    if n < 2:
        return 0.0, 0.0, (x, y), 0, 0.0, 0.0

    _, index, _, _, t = _project_on_path(x, y, path, index_hint)
    carrot_x, carrot_y, _ = _carrot_point(path, index, t, cfg['lookahead'])

    # remaining distance from the projection point to the last vertex
    px = path[index][0] + (path[index + 1][0] - path[index][0]) * t
    py = path[index][1] + (path[index + 1][1] - path[index][1]) * t
    remaining = math.hypot(path[index + 1][0] - px,
                           path[index + 1][1] - py)
    for j in range(index + 1, n - 1):
        remaining += math.hypot(path[j + 1][0] - path[j][0],
                                path[j + 1][1] - path[j][1])

    dx, dy = carrot_x - x, carrot_y - y
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0, 0.0, (carrot_x, carrot_y), index, remaining, 0.0
    alpha = normalize_angle(math.atan2(dy, dx) - yaw)

    # slow down in corners and near the goal
    v = cfg['max_linear']
    if cfg['slow_angle'] > 0.0:
        ratio = clamp(abs(alpha) / cfg['slow_angle'], 0.0, 1.0)
        v *= 1.0 - (1.0 - cfg['slow_factor']) * ratio
    if remaining < cfg['end_slowdown']:
        v = min(v, max(cfg['min_linear'], 0.6 * remaining))

    kappa = 2.0 * math.sin(alpha) / max(L, 0.05)
    w = clamp(v * kappa, -cfg['max_angular'], cfg['max_angular'])

    goal_x, goal_y = path[-1]
    if math.hypot(goal_x - x, goal_y - y) <= cfg['goal_tolerance']:
        return 0.0, 0.0, (goal_x, goal_y), n - 2, 0.0, alpha
    return v, w, (carrot_x, carrot_y), index, remaining, alpha


FOLLOW_STATE = 'FOLLOW'
TURN_STATE = 'TURN'
CORNER_STATE = 'CORNER'
RECOVER_STATE = 'RECOVER'

DEFAULT_WALL_CFG = {
    # longitudinal: adaptive speed
    'front_stop': 0.42,        # m   start a turn when a wall is this close
    'speed': 0.22,             # m/s cruise speed (also the upper limit)
    'speed_min': 0.05,         # m/s lower limit while following
    'speed_clear_distance': 1.00,  # m   front clearance for full (cruise) speed
    'speed_clear_min': 0.70,   # speed factor at front_stop (1.0 = no reduction)
    'curve_slow_gain': 0.75,   # 1   speed reduction per unit of |w|/max_w
    'curve_slow_min': 0.35,    # speed factor when steering at max_w
    'lateral_slow_gain': 0.6,  # 1   extra slow down when hugging the left wall
    'lateral_slow_min': 0.5,   # speed factor when very close to the left wall
    'accel': 0.7,              # m/s^2  acceleration limit (smooth starts)
    'decel': 1.2,              # m/s^2  deceleration limit (smooth stops)
    'w_accel': 6.0,            # rad/s^2 angular command slew limit
    # lateral (left wall), kept gentle so the heading stays corridor-aligned
    'target_left': 0.34,       # m
    'kp_lat': 0.6,             # 1/s
    'lat_limit': 0.12,         # m   clamp of the distance error fed to the controller
    'kp_yaw': 1.5,             # 1/s
    'max_w': 0.9,              # rad/s
    # turning
    'open_distance': 0.55,     # m   a side sector above this is an opening
    'turn_angle': 1.5708,      # rad 90 deg
    'uturn_angle': 3.1416,     # rad 180 deg
    'kp_turn': 2.5,            # 1/s
    'max_turn_rate': 1.2,      # rad/s
    'turn_tolerance': 0.05,    # rad
    'blocked_ticks': 3,        # debounce for "wall ahead"
    # "dead end" (front, left and right all blocked) is the expensive call: it
    # spins the robot 180 deg.  A single frame of a wall-hugging pose used to be
    # enough, and with the wide +-20 deg front fan the side wall itself can read
    # as a wall ahead (0.36 m in a clear corridor).  Hold the verdict for a few
    # ticks so that a transient pose cannot trigger the U-turn.
    'deadend_ticks': 8,        # 0.4 s at 20 Hz before the U-turn is allowed
    'turn_cooldown': 0.4,      # s   ignore new triggers right after a turn
    # A pivot out of a dead end does not guarantee a better heading: in a
    # 0.8 m cell the wall can already be 0.30 m in front again, and re-deciding
    # from the noisy side sectors makes the robot swing +-90 deg for ever
    # (observed once in 10 runs: 13 turns, 65.6 s).  Keep rotating the same way
    # until the front is clear, up to one full circle, then give up.
    'turn_scan_after': 1.0,    # 1 = extend a dead-end turn until the front is clear
    'turn_max_span': 2 * math.pi,  # rad  stop extending after a full circle
    # A robot wedged in a narrow slot can rotate for ever without going
    # anywhere: three pivots inside a 0.20 m circle means the geometry is too
    # tight for turning, so back out instead of spinning.
    'turn_loop_count': 3,      # pivots within one spot that count as a loop
    'turn_loop_distance': 0.20,  # m  radius that defines "one spot"
    # doorway corner cutting (arc instead of a pivot turn)
    'corner_enabled': 1.0,     # 0 = pure pivot turns (old behaviour)
    'corner_half_width': 0.35,  # m   distance from corridor centre to the side wall
    'corner_entry_back': 0.30,  # m   gate centre is this far before the inner corner
    'corner_exit_length': 0.35,  # m   length of the virtual path inside the new corridor
    'corner_radius': 0.22,     # m   rounding radius of the virtual path
    'corner_samples': 6,       # arc samples per rounded vertex
    'corner_speed': 0.30,      # m/s speed while tracking the corner path
    'corner_lookahead': 0.22,  # m   pure pursuit lookahead on the corner path
    'corner_max_rate': 1.2,    # rad/s angular limit while tracking the corner
    # straight-line (diagonal) corner entry: cut the junction on the chord
    # instead of following the two perpendicular legs
    'corner_diagonal': 1.0,       # 1 = diagonal shortcut, 0 = rounded L path
    'corner_min_clearance': 0.17,  # m needed between path and the inner corner
    'corner_diagonal_radius': 0.10,  # m small arc where the chord is too tight
    'corner_tolerance': 0.06,  # m   remaining distance that ends the corner
    'corner_abort_distance': 0.12,  # m  abort to a pivot turn below this clearance
    'corner_max_time': 4.0,    # s   abort the corner if it takes longer
    # chained corners ("Z" / S-bends): two opposite turns close together
    'corner_chain_enabled': 1.0,   # 0 = treat every corner on its own
    'corner_chain_distance': 1.00,  # m  a corner this soon after the last is chained
    'corner_chain_entry_back': 0.30,  # m  gate centre, same as a normal corner
    'corner_chain_radius': 0.20,   # m  radius for the chained S
    'corner_chain_speed': 0.32,    # m/s no stop between the two turns of a Z
    # --- robustness for the 5 Hz LDS-01 of the stock TurtleBot3 -------------
    # A scan only arrives every 0.2 s, so a single stale sample must not be
    # able to abort an arc (that used to throw the robot into pivot turns in
    # the middle of a junction), and a robot that has physically stopped must
    # be able to back out instead of pushing against a wall forever.
    'corner_abort_ticks': 4,      # consecutive unsafe ticks before the arc is dropped
    'corner_abort_grace': 0.20,   # s   clearance test is ignored right after the trigger
    'stuck_enabled': 1.0,         # 1 = detect "no progress" and back out
    'stuck_window': 1.20,         # s   length of the progress test
    'stuck_distance': 0.03,       # m   displacement below which we count as stuck
    'stuck_speed_gate': 0.05,     # m/s the command must ask for real motion
    'recover_speed': 0.10,        # m/s reverse speed while backing out
    'recover_time': 0.90,         # s   how long to back out
    'recover_cooldown': 0.80,     # s   no new turn right after a recovery
}


class WallFollowerController:
    """Latched left-wall following state machine (pure Python, no ROS).

    ``step(front, left, right, yaw, dt, pose=None)`` returns
    ``(v, w, state)``.

    Three ideas make this stable in a 0.8 m grid maze:

    * **Latched turns.**  A corner turn fixes a target yaw and rotates to it.
      Re-deciding from the sectors at 20 Hz oscillates in front of every wall.
    * **Heading hold.**  While driving, the angular command is dominated by a
      term that keeps the robot aligned with the corridor it is in, plus a
      gentle term that pushes it back to ``target_left``.  A pure distance
      controller veers off by tens of degrees next to every doorway and then
      every 90 deg turn is off by that amount.
    * **Corner cutting (``CORNER``).**  When a 90 deg opening is detected and
      ``pose`` is known, a short virtual path is built through the doorway and
      tracked with pure pursuit, so the robot drives an arc instead of
      stopping to rotate in place.  Dead ends and unsafe geometry fall back to
      a latched pivot turn.

    ``front``, ``left`` and ``right`` are the sector distances in metres
    (see :func:`maze_control.utils.scan_sectors`); ``yaw`` is the odometry
    heading in radians; ``pose`` is ``(x, y)`` in metres.
    """

    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULT_WALL_CFG)
        if cfg:
            self.cfg.update(cfg)
        self.state = FOLLOW_STATE
        self.target_yaw = 0.0
        self.desired_yaw = None
        self.turn_dir = 1.0
        self.blocked = 0
        self.turns = 0
        self.cooldown = 0.0
        self.last_event = None
        self.last_reason = ''
        # smoothed commands (rate limited) so that the robot does not jerk
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        # corner cutting: virtual path tracked with pure pursuit
        self.corner_path = None
        self.corner_index = 0
        self.corner_time = 0.0
        self.since_corner = 1e9      # m travelled since the last corner ended
        self.prev_corner_delta = 0.0
        self.chained = False
        self.corner_unsafe = 0       # debounce counter for the arc abort
        self.deadend_ticks = 0       # debounce counter for the dead-end U-turn
        self.turn_total = 0.0        # rotation already commanded in this turn
        self.turn_escape = False     # the turn was started from a dead end
        self._loop_origin = None     # pose of the current pivot burst
        self._loop_turns = 0
        # "no progress" supervision (see _stuck / _recover_command)
        self._clock = 0.0
        self._ref = None
        self._recover_left = 0.0
        self.corner_pp = dict(DEFAULT_PP_CFG)
        self.corner_pp.update({
            'lookahead': self.cfg['corner_lookahead'],
            'min_lookahead': self.cfg['corner_lookahead'],
            'max_lookahead': self.cfg['corner_lookahead'],
            'max_linear': self.cfg['corner_speed'],
            'min_linear': self.cfg['corner_speed'],
            'max_angular': self.cfg['corner_max_rate'],
            'slow_angle': 0.6,
            'slow_factor': 0.6,
            'end_slowdown': 0.0,
            'goal_tolerance': 0.02,
        })

    def reset(self):
        self.state = FOLLOW_STATE
        self.blocked = 0
        self.turns = 0
        self.cooldown = 0.0
        self.desired_yaw = None
        self.last_event = None
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.corner_path = None
        self.corner_index = 0
        self.corner_time = 0.0
        self.corner_unsafe = 0          # debounce counter for the arc abort
        self.deadend_ticks = 0          # debounce counter for the dead-end U-turn
        self.turn_total = 0.0           # rotation already commanded in this turn
        self.turn_escape = False        # the turn was started from a dead end
        self._loop_origin = None        # pose of the current pivot burst
        self._loop_turns = 0
        self.since_corner = 1e9
        self.prev_corner_delta = 0.0
        self.chained = False
        # "no progress" supervision (see _stuck / _recover_command)
        self._clock = 0.0
        self._ref = None
        self._recover_left = 0.0

    def step(self, front, left, right, yaw, dt=0.05, pose=None,
             front_wide=None):
        """One control tick.

        ``front`` is the narrow (+-6 deg) cone and decides whether the way
        ahead is blocked; ``front_wide`` is the wide (+-20 deg) fan used for the
        speed ramp and the corner geometry and defaults to ``front`` so that
        callers that only have one front distance keep working.
        """
        cfg = self.cfg
        fwd = front if front_wide is None else front_wide
        self.last_event = None
        self._clock += dt
        self.since_corner += abs(self.v_cmd) * dt
        if self.desired_yaw is None:
            self.desired_yaw = yaw
        if self.cooldown > 0.0:
            self.cooldown -= dt

        if self.state == RECOVER_STATE:
            return self._recover_command(dt)

        if self.state == CORNER_STATE:
            # At 5 Hz the clearance test sees a scan that is up to 0.2 s old,
            # so one low reading must not kill the arc: it has to persist for
            # corner_abort_ticks ticks and the grace time has to be over.
            close = min(front, left, right) < cfg['corner_abort_distance']
            self.corner_unsafe = self.corner_unsafe + 1 if close else 0
            unsafe = (pose is None
                      or self.corner_time > cfg['corner_max_time']
                      or (self.corner_unsafe >= cfg['corner_abort_ticks']
                          and self.corner_time >= cfg['corner_abort_grace']))
            if unsafe:
                # no room / no odometry / too slow: rotate in place instead
                self.state = TURN_STATE
                self.corner_path = None
                self.v_cmd = 0.0
                self.last_reason += ' (corner aborted)'
                return 0.0, self._slew_w(self.turn_dir * cfg['max_turn_rate'], dt), self.state
            return self._corner_command(pose, yaw, dt)

        if self.state == TURN_STATE:
            err = normalize_angle(self.target_yaw - yaw)
            if abs(err) <= cfg['turn_tolerance']:
                # Reached the latched heading.  If this turn was started from a
                # dead end and the new heading is blocked as well, rotating back
                # and forth between two dead ends is what used to happen; keep
                # going the same way instead (one cell is only 0.8 m, so a 90
                # deg step frequently lands on another wall).
                if (cfg['turn_scan_after'] >= 0.5 and self.turn_escape
                        and front < cfg['front_stop']
                        and self.turn_total + cfg['turn_angle']
                        <= cfg['turn_max_span'] + 1e-6):
                    self.turn_total += cfg['turn_angle']
                    self.target_yaw = normalize_angle(
                        self.target_yaw + self.turn_dir * cfg['turn_angle'])
                    self.last_event = 'turn_extend'
                    self.v_cmd = 0.0
                    return (0.0,
                            self._slew_w(cfg['max_turn_rate'] * self.turn_dir, dt),
                            self.state)
                self.state = FOLLOW_STATE
                self.blocked = 0
                self.desired_yaw = self.target_yaw
                self.last_event = 'turn_done'
                self.v_cmd = 0.0
                self.w_cmd = 0.0
                return 0.0, 0.0, self.state
            w = clamp(cfg['kp_turn'] * err,
                      -cfg['max_turn_rate'], cfg['max_turn_rate'])
            self.v_cmd = 0.0
            return 0.0, self._slew_w(w, dt), self.state

        if front < cfg['front_stop']:
            self.blocked += 1
        else:
            self.blocked = 0
            self.deadend_ticks = 0

        if self.blocked >= cfg['blocked_ticks'] and self.cooldown <= 0.0:
            # Both sides closed means a U-turn.  That is the most expensive
            # decision the follower makes, and in a 0.7 m corridor a wall-
            # hugging pose can make an open corridor look closed, so require
            # the picture to hold still for deadend_ticks ticks first.
            if (left < cfg['open_distance'] and right < cfg['open_distance']):
                self.deadend_ticks += 1
                if self.deadend_ticks < int(cfg['deadend_ticks']):
                    self.desired_yaw = yaw
                    self.v_cmd = 0.0
                    self.last_event = 'deadend_hold'
                    return 0.0, self._slew_w(0.0, dt), self.state
            else:
                self.deadend_ticks = 0
            if self._turn_loop(pose):
                self._start_recover('turn loop', 'turn_loop')
                return self._recover_command(dt)
            delta = self._start_turn(front, left, right, yaw)
            path = None
            if (cfg['corner_enabled'] >= 0.5 and pose is not None
                    and abs(abs(delta) - cfg['turn_angle']) < 1e-3):
                path = self._build_corner_path(pose, yaw, fwd, delta)
            if path is not None:
                self.corner_path = path
                self.corner_index = 0
                self.corner_time = 0.0
                self.corner_unsafe = 0
                self.state = CORNER_STATE
                self.last_reason += (' (chained S)' if self.chained
                                     else ' (corner arc)')
                return self._corner_command(pose, yaw, dt)
            return 0.0, self.turn_dir * cfg['max_turn_rate'], self.state

        err_lat = clamp(left - cfg['target_left'],
                        -cfg['lat_limit'], cfg['lat_limit'])
        err_yaw = normalize_angle(self.desired_yaw - yaw)
        w = clamp(cfg['kp_yaw'] * err_yaw + cfg['kp_lat'] * err_lat,
                  -cfg['max_w'], cfg['max_w'])
        w = self._slew_w(w, dt)
        v = self._adaptive_speed(fwd, left, w)
        v = self._slew_v(v, dt)
        if self._stuck(pose, v, w):
            self._start_recover()
            return self._recover_command(dt)
        return v, w, self.state

    # ---------------------------------------------------- stuck supervision
    def _stuck(self, pose, v, w):
        """True when the commands ask for motion but the pose does not move.

        A real TurtleBot3 can wedge itself into a doorway (wheel slip, a
        slightly late scan), after which the follower keeps pushing against
        the wall for ever.  Comparing the odometry pose against a reference
        sample that is re-armed every ``stuck_window`` seconds detects exactly
        that case and nothing else (a sliding queue would slide along with the
        comparison and never fire).
        """
        cfg = self.cfg
        if cfg['stuck_enabled'] < 0.5 or pose is None:
            return False
        if v < cfg['stuck_speed_gate'] or abs(w) > 0.6:
            # pivoting on purpose, or stopped on purpose: not a jam
            self._ref = None
            return False
        if self._ref is None:
            self._ref = (self._clock, pose[0], pose[1])
            return False
        t0, x0, y0 = self._ref
        if self._clock - t0 < cfg['stuck_window']:
            return False
        # one full window has passed: re-arm the reference, then judge
        self._ref = (self._clock, pose[0], pose[1])
        return math.hypot(pose[0] - x0, pose[1] - y0) < cfg['stuck_distance']

    def _start_recover(self, reason='stuck: backing out', event='stuck'):
        cfg = self.cfg
        self.state = RECOVER_STATE
        self._recover_left = cfg['recover_time']
        self._ref = None
        self.corner_path = None
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.blocked = 0
        self.last_event = event
        self.last_reason = reason

    def _recover_command(self, dt):
        cfg = self.cfg
        self._recover_left -= dt
        if self._recover_left <= 0.0:
            self.state = FOLLOW_STATE
            self.blocked = 0
            self.cooldown = cfg['recover_cooldown']
            self.desired_yaw = None
            self.last_event = 'recover_done'
            return 0.0, 0.0, self.state
        return (self._slew_v(-cfg['recover_speed'], dt),
                self._slew_w(0.0, dt), self.state)

    def _adaptive_speed(self, front, left, w):
        """Speed that follows the free space in front of the robot.

        Three independent factors, multiplied together, all in (0, 1]:

        * **clearance** - slow down when the wall ahead gets close, so the
          robot does not have to brake abruptly at a corner;
        * **curvature** - slow down while steering hard, which keeps the
          lateral correction inside the corridor (and makes the motion look
          much less "wobbly");
        * **lateral** - slow down when the left wall is closer than the
          target distance, because a tight corridor needs more care.

        The result is clamped to ``[speed_min, speed]``.
        """
        cfg = self.cfg
        clear_span = max(cfg['speed_clear_distance'] - cfg['front_stop'], 1e-3)
        clear = clamp((front - cfg['front_stop']) / clear_span, 0.0, 1.0)
        f_clear = cfg['speed_clear_min'] + (1.0 - cfg['speed_clear_min']) * clear

        steer = clamp(abs(w) / max(cfg['max_w'], 1e-3), 0.0, 1.0)
        f_curve = clamp(1.0 - cfg['curve_slow_gain'] * steer,
                        cfg['curve_slow_min'], 1.0)

        err_lat = left - cfg['target_left']
        if err_lat >= 0.0:
            f_lat = 1.0
        else:
            f_lat = clamp(
                1.0 + cfg['lateral_slow_gain'] * err_lat / max(cfg['target_left'], 1e-3),
                cfg['lateral_slow_min'], 1.0)

        v = cfg['speed'] * f_clear * f_curve * f_lat
        return clamp(v, cfg['speed_min'], cfg['speed'])

    def _slew_v(self, target, dt):
        """Rate limit the linear command (acceleration / braking)."""
        cfg = self.cfg
        rate = cfg['accel'] if target > self.v_cmd else cfg['decel']
        step = rate * dt
        self.v_cmd += clamp(target - self.v_cmd, -step, step)
        return self.v_cmd

    def _slew_w(self, target, dt):
        """Rate limit the angular command so turns start and stop smoothly."""
        step = self.cfg['w_accel'] * dt
        self.w_cmd += clamp(target - self.w_cmd, -step, step)
        return self.w_cmd

    def _turn_loop(self, pose):
        """True when the robot keeps pivoting on the spot without escaping.

        In a 0.12 m wide slot a 180 deg pivot can end facing another wall a few
        centimetres away, so the follower decides "dead end" again and rotates
        back.  Counting the pivots that happen inside a small circle turns that
        (which looks like ordinary turning from inside the state machine) into a
        backup manoeuvre.
        """
        cfg = self.cfg
        if pose is None or cfg['turn_loop_count'] <= 0:
            return False
        if self._loop_origin is None:
            self._loop_origin = pose
            self._loop_turns = 0
        moved = math.hypot(pose[0] - self._loop_origin[0],
                           pose[1] - self._loop_origin[1])
        if moved > cfg['turn_loop_distance']:
            self._loop_origin = pose
            self._loop_turns = 0
            return False
        self._loop_turns += 1
        if self._loop_turns >= int(cfg['turn_loop_count']):
            self._loop_origin = None
            self._loop_turns = 0
            return True
        return False

    def _start_turn(self, front, left, right, yaw):
        cfg = self.cfg
        left_open = left > cfg['open_distance']
        right_open = right > cfg['open_distance']
        if left_open and not right_open:
            delta, reason = cfg['turn_angle'], 'left opening'
        elif right_open and not left_open:
            delta, reason = -cfg['turn_angle'], 'right opening'
        elif left_open and right_open:
            delta, reason = cfg['turn_angle'], 'junction, take left'
        else:
            delta, reason = cfg['uturn_angle'], 'dead end'

        self.last_reason = reason
        self.turn_delta = delta
        self.turn_dir = 1.0 if delta > 0 else -1.0
        self.turn_total = abs(delta)
        self.turn_escape = (reason == 'dead end')
        self.turns += 1
        self.state = TURN_STATE
        self.blocked = 0
        self.cooldown = cfg['turn_cooldown']
        self.target_yaw = normalize_angle(self.desired_yaw + delta)
        self.last_event = 'turn_start'
        return delta

    def _build_corner_path(self, pose, yaw, front, delta):
        """Virtual path that cuts diagonally through the doorway.

        The wall ahead is ``front`` metres away; the side wall of the corridor
        is ``corner_half_width`` to the side, so the two meet at the inner
        corner of the turn ``K``.  The doorway centre ``G`` sits half a doorway
        back from ``K`` (the gap in the side wall), and ``E`` is inside the new
        corridor.  Rounding ``[pose, G, E]`` gives a smooth arc whose chord
        uses the full doorway width instead of pivoting at ``K``.

        Returns ``None`` when the geometry does not make sense (the caller
        then falls back to a pivot turn).
        """
        cfg = self.cfg
        if not (0.05 < front < 2.0):
            return None
        # A corner that follows the previous one almost immediately and turns
        # the other way is an S-bend ("Z" corridor).  There is no straight
        # between the two turns there: start the arc at once and use a wider
        # radius so the robot flows through instead of stopping to re-latch.
        self.chained = (cfg['corner_chain_enabled'] >= 0.5
                        and self.since_corner < cfg['corner_chain_distance']
                        and self.prev_corner_delta * delta < 0.0)
        entry_back = (cfg['corner_chain_entry_back'] if self.chained
                      else cfg['corner_entry_back'])
        radius = cfg['corner_chain_radius'] if self.chained else cfg['corner_radius']
        speed = cfg['corner_chain_speed'] if self.chained else cfg['corner_speed']
        self.corner_pp['max_linear'] = speed
        self.corner_pp['min_linear'] = speed
        x, y = pose
        fx, fy = math.cos(yaw), math.sin(yaw)
        side = 1.0 if delta > 0 else -1.0
        sx, sy = -side * fy, side * fx          # unit vector towards the turn
        h = cfg['corner_half_width']

        kx = x + front * fx + h * sx            # inner corner of the turn
        ky = y + front * fy + h * sy
        gx = kx - entry_back * fx               # doorway centre
        gy = ky - entry_back * fy
        ex = gx + cfg['corner_exit_length'] * sx  # inside the new corridor
        ey = gy + cfg['corner_exit_length'] * sy

        if cfg['corner_diagonal'] >= 0.5:
            # Option B: leave the corner on the chord from the current pose to
            # the exit point.  Only when that chord would graze the inner
            # corner do we add one waypoint that pushes it outwards, rounded by
            # a small arc ("small radius adjustment" for tight spots).
            need = cfg['corner_min_clearance']
            dist = _point_segment_distance(kx, ky, x, y, ex, ey)
            if dist >= need + cfg['corner_diagonal_radius']:
                pts = _line_path(x, y, ex, ey, int(cfg['corner_samples']))
            else:
                # closest point on the chord to K, then step outwards
                t = _project_param(kx, ky, x, y, ex, ey)
                px, py = x + (ex - x) * t, y + (ey - y) * t
                dx, dy = px - kx, py - ky
                norm = math.hypot(dx, dy) or 1.0
                out = need + cfg['corner_diagonal_radius'] + 0.03
                mx, my = kx + dx / norm * out, ky + dy / norm * out
                pts = smooth_path([(x, y), (mx, my), (ex, ey)],
                                  cfg['corner_diagonal_radius'],
                                  int(cfg['corner_samples']))
        else:
            pts = smooth_path([(x, y), (gx, gy), (ex, ey)],
                              radius, int(cfg['corner_samples']))
        if len(pts) < 3:
            return None
        return pts

    def _corner_command(self, pose, yaw, dt):
        cfg = self.cfg
        self.corner_time += dt
        x, y = pose
        v, w, _carrot, index, remaining, _alpha = pure_pursuit_command(
            x, y, yaw, self.corner_path, self.corner_pp, self.corner_index)
        self.corner_index = index
        if remaining <= cfg['corner_tolerance']:
            self.state = FOLLOW_STATE
            self.desired_yaw = self.target_yaw
            self.corner_path = None
            self.blocked = 0
            self.since_corner = 0.0
            self.prev_corner_delta = self.turn_delta
            self.last_event = 'corner_done'
            self.v_cmd = v
            self.w_cmd = w
            return v, w, self.state
        return self._slew_v(v, dt), self._slew_w(w, dt), self.state
