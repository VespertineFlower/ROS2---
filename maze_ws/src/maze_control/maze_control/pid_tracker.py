"""Feedforward + PID path follower (no ROS, fully unit-testable).

The pure-pursuit tracker only reacts to where the carrot point currently is,
so it starts slowly and wobbles around the arcs.  This controller instead

* feeds the planned curvature forward   (``omega_ff = v * kappa(s)``),
* closes the loop on the *cross track error* with a PID  (integral term
  removes the steady offset caused by wheel/odometry bias),
* damps the heading error with a proportional term,
* and follows the planned speed profile with a slew limiter.

Sign convention: the path is given as a polyline from start to goal, the
lateral error ``e`` is positive when the robot is to the **left** of the
path, and a positive ``omega`` turns the robot counter-clockwise.
"""

import math

import numpy as np

from .utils import clamp, normalize_angle

DEFAULT_PID_CFG = {
    'kp_lat': 4.0,          # 1/(m*s)  cross track P
    'ki_lat': 0.30,         # 1/(m*s^2) cross track I
    'kd_lat': 0.10,         # 1/(m*s)  cross track D  (on measurement)
    'i_limit': 0.15,        # m*s      integrator clamp (anti windup)
    'kp_theta': 2.0,        # 1/s      heading error P
    'max_omega': 2.0,       # rad/s    steering saturation
    'lookahead_s': 0.12,    # m        where on the profile we read the speed
    'slow_gain': 0.6,       # speed reduction per metre of lateral error
    'slow_max': 0.45,       # never reduce the speed more than this
    'max_linear': 0.40,     # m/s      absolute cap
    'accel': 1.10,          # m/s^2    longitudinal slew up
    'decel': 1.80,          # m/s^2    longitudinal slew down
    'curv_smooth': 5,       # samples  moving average window for curvature
}


def path_geometry(points, cfg=None):
    """Pre-compute arc length, unit tangents and curvature along a polyline."""
    cfg = dict(DEFAULT_PID_CFG, **(cfg or {}))
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        raise ValueError('path needs at least two points')
    seg = np.diff(pts, axis=0)
    lengths = np.linalg.norm(seg, axis=1)
    lengths = np.maximum(lengths, 1e-9)
    tangents = seg / lengths[:, None]
    s = np.concatenate([[0.0], np.cumsum(lengths)])
    # discrete curvature from the angle change between consecutive segments
    kappa = np.zeros(len(pts))
    for i in range(1, len(pts) - 1):
        a, b = tangents[i - 1], tangents[i]
        cross = a[0] * b[1] - a[1] * b[0]
        dot = clamp(float(a[0] * b[0] + a[1] * b[1]), -1.0, 1.0)
        dtheta = math.atan2(cross, dot)
        ds = 0.5 * (lengths[i - 1] + lengths[i])
        kappa[i] = dtheta / max(ds, 1e-9)
    kappa[0], kappa[-1] = kappa[1], kappa[-2]
    w = int(cfg['curv_smooth'])
    if w >= 3:
        kernel = np.ones(w) / float(w)
        kappa = np.convolve(np.pad(kappa, (w // 2, w // 2), mode='edge'),
                            kernel, mode='valid')[:len(kappa)]
    return {'points': pts, 's': s, 'tangents': tangents, 'kappa': kappa,
            'length': float(s[-1])}


def project(geom, x, y, hint=0):
    """Closest point on the path: (s, signed lateral error, heading, kappa)."""
    pts = geom['points']
    n = len(pts)
    i0 = max(0, min(int(hint), n - 2))
    def _scan(lo, hi, best):
        for i in range(lo, hi):
            a, b = pts[i], pts[i + 1]
            d = b - a
            L2 = float(d[0] * d[0] + d[1] * d[1])
            if L2 < 1e-12:
                continue
            t = clamp(((x - a[0]) * d[0] + (y - a[1]) * d[1]) / L2, 0.0, 1.0)
            px, py = a[0] + t * d[0], a[1] + t * d[1]
            dist = math.hypot(x - px, y - py)
            if best is None or dist < best[0]:
                best = (dist, i, t, px, py)
        return best

    # Search a window around the hint first; only trust it while the robot is
    # still close to the path.  On a doubled-back route the window can hold a
    # stale branch, and using it makes the lateral error jump by half a metre,
    # so fall back to the whole path whenever the window looks suspicious.
    best = _scan(i0, min(n - 1, i0 + 40), None)
    if best is None or best[0] > 0.25:
        best = _scan(0, n - 1, best)
    if best is None:
        raise ValueError('degenerate path')
    _, i, t, _px, _py = best
    a, b = pts[i], pts[i + 1]
    tang = geom['tangents'][i]
    left = np.array([-tang[1], tang[0]])
    e = float((x - a[0]) * left[0] + (y - a[1]) * left[1])
    heading = math.atan2(tang[1], tang[0])
    kappa = float(geom['kappa'][i] * (1.0 - t) + geom['kappa'][i + 1] * t)
    return geom['s'][i] + t * (geom['s'][i + 1] - geom['s'][i]), e, heading, kappa


def speed_at(geom, speeds, s):
    """Interpolate the planned speed profile at arc length ``s``."""
    arr = np.asarray(speeds, dtype=float)
    if len(arr) != len(geom['s']):
        frac = s / max(geom['length'], 1e-6)
        idx = int(frac * (len(arr) - 1))
        return float(arr[min(len(arr) - 1, max(0, idx))])
    return float(np.interp(s, geom['s'], arr))


class PidPathFollower:
    """Feedforward curvature + PID cross-track control along a planned path."""

    def __init__(self, points, speeds=None, cfg=None):
        self.cfg = dict(DEFAULT_PID_CFG, **(cfg or {}))
        self.geom = path_geometry(points, self.cfg)
        if speeds is None:
            speeds = np.full(len(points), self.cfg['max_linear'])
        self.speeds = np.asarray(speeds, dtype=float)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.e_prev = 0.0
        self.v_cmd = 0.0
        self.hint = 0
        self.finished = False
        self.last = {'s': 0.0, 'e': 0.0, 'theta': 0.0, 'kappa': 0.0, 'v': 0.0,
                     'w': 0.0}

    def update(self, x, y, yaw, dt=0.05, v_meas=None, stop_at_end=True):
        """One control step -> (v_cmd, w_cmd, state dict)."""
        cfg = self.cfg
        dt = max(float(dt), 1e-3)
        s, e, heading, kappa = project(self.geom, x, y, self.hint)
        self.hint = max(0, min(int(s / max(self.geom['length'], 1e-6) * len(self.geom['s'])),
                               len(self.geom['s']) - 2))
        theta = normalize_angle(heading - yaw)   # path heading - robot heading

        # ---- longitudinal -------------------------------------------------
        s_look = min(s + cfg['lookahead_s'], self.geom['length'])
        v_ref = speed_at(self.geom, self.speeds, s_look)
        slow = min(cfg['slow_max'], cfg['slow_gain'] * abs(e))
        v_ref = max(0.0, v_ref * (1.0 - slow))
        v_ref = min(v_ref, cfg['max_linear'])
        remaining = self.geom['length'] - s
        if stop_at_end:
            v_ref = min(v_ref, max(0.0, 0.15 + 1.5 * remaining))
        self.v_cmd = clamp(v_ref, self.v_cmd - cfg['decel'] * dt,
                           self.v_cmd + cfg['accel'] * dt)
        self.v_cmd = max(0.0, self.v_cmd)

        # ---- lateral: PID on cross track + P on heading -------------------
        d_e = (e - self.e_prev) / dt
        self.e_prev = e
        self.integral = clamp(self.integral + e * dt,
                              -cfg['i_limit'], cfg['i_limit'])
        pid = (cfg['kp_lat'] * e + cfg['ki_lat'] * self.integral
               + cfg['kd_lat'] * d_e)
        omega = self.v_cmd * kappa + cfg['kp_theta'] * theta - pid
        omega = clamp(omega, -cfg['max_omega'], cfg['max_omega'])

        if remaining < 0.06:
            self.finished = True
            self.v_cmd = 0.0
            omega = 0.0
            self.integral = 0.0
        self.last = {'s': s, 'e': e, 'theta': theta, 'kappa': kappa,
                     'v': self.v_cmd, 'w': omega}
        return self.v_cmd, omega, dict(self.last)
