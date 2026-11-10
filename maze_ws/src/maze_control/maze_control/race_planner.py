"""Two-pass "map first, race second" path planner (pure Python, no ROS).

Pass 1 (explore) records ``/scan`` + ``/odom`` into an occupancy grid; pass 2
(race) plans the fastest trajectory through the free space:

1. A* on the inflated grid -> shortest collision free polyline,
2. visibility based shortcutting + Douglas-Peucker -> few vertices,
3. every remaining vertex is replaced by an arc whose radius is the largest
   one the local clearance allows (corner cutting with a map),
4. the polyline is resampled and a time-optimal speed profile is computed
   from the curvature (lateral acceleration limit) with forward/backward
   acceleration limits.

The result is a list of points plus the speed allowed at each of them, which
the ROS nodes drive with the same pure pursuit law used elsewhere.
"""

import math
from collections import deque

import numpy as np

try:  # scipy is available on the target image, but keep a fallback
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover
    distance_transform_edt = None


DEFAULT_CFG = {
    'res': 0.05,            # m   grid resolution
    'bounds': (-0.30, 4.30, -0.30, 4.30),   # xmin, xmax, ymin, ymax
    'obstacle_radius': 0.15,   # m  planning inflation (half diagonal + margin)
    'clearance_min': 0.06,     # m  A* refuses cells closer than this
    'v_max': 0.42,         # m/s top speed
    'a_lat': 0.60,         # m/s^2 lateral acceleration limit
    'a_acc': 0.55,         # m/s^2
    'a_dec': 0.80,         # m/s^2
    'corner_margin': 0.10,  # m  distance kept between arc and the walls
    'r_min': 0.10,         # m  smallest arc radius used by round_corners
    'r_max': 0.45,         # m  largest corner radius
    'max_yaw_rate': 1.40,  # rad/s  cap on v * kappa (tracker saturates at 2.0)
    'smooth_iterations': 2,  # Chaikin passes: rounds the 45 deg A* facets
    'curv_smooth': 5,      # samples  moving average for the curvature estimate
    'simplify_tol': 0.08,  # m  Douglas-Peucker tolerance before smoothing
    'sample_step': 0.02,   # m  resampling step
    'arc_clearance': 0.15,  # m  distance the corner arcs keep to the walls
    'restrict_to_maze': 1.0,  # plan inside the wall outline only (see below)
    'goal_pocket': 0.35,   # m  radius kept free around a goal outside the maze
}


class GridMap:
    """Occupancy grid with axis aligned cells and a clearance field."""

    def __init__(self, occupied, res, ox, oy):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.res = float(res)
        self.ox = float(ox)
        self.oy = float(oy)
        self.nx = self.occupied.shape[1]
        self.ny = self.occupied.shape[0]
        self.clearance = self._clearance()

    # ------------------------------------------------------------------ build
    @classmethod
    def from_walls(cls, segments, cfg=None):
        cfg = dict(DEFAULT_CFG, **(cfg or {}))
        res = cfg['res']
        xmin, xmax, ymin, ymax = cfg['bounds']
        nx = int(round((xmax - xmin) / res)) + 1
        ny = int(round((ymax - ymin) / res)) + 1
        occ = np.zeros((ny, nx), dtype=bool)
        for (cx, cy, sx, sy, _name) in segments:
            i0 = int(round((cx - sx / 2.0 - xmin) / res))
            i1 = int(round((cx + sx / 2.0 - xmin) / res))
            j0 = int(round((cy - sy / 2.0 - ymin) / res))
            j1 = int(round((cy + sy / 2.0 - ymin) / res))
            occ[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = True
        # the outer frame of the map counts as occupied too
        occ[0, :] = occ[-1, :] = True
        occ[:, 0] = occ[:, -1] = True
        return cls(occ, res, xmin, ymin)

    @classmethod
    def from_scans(cls, points, cfg=None):
        """Build a grid by marking every recorded laser hit as occupied."""
        cfg = dict(DEFAULT_CFG, **(cfg or {}))
        res = cfg['res']
        xmin, xmax, ymin, ymax = cfg['bounds']
        nx = int(round((xmax - xmin) / res)) + 1
        ny = int(round((ymax - ymin) / res)) + 1
        occ = np.zeros((ny, nx), dtype=bool)
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        i = np.rint((pts[:, 0] - xmin) / res).astype(int)
        j = np.rint((pts[:, 1] - ymin) / res).astype(int)
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        occ[j[ok], i[ok]] = True
        occ[0, :] = occ[-1, :] = True
        occ[:, 0] = occ[:, -1] = True
        return cls(occ, res, xmin, ymin)

    def _clearance(self):
        if distance_transform_edt is None:  # pragma: no cover
            return np.zeros_like(self.occupied, dtype=float)
        free = ~self.occupied
        dist = distance_transform_edt(free)
        return dist * self.res

    # ------------------------------------------------------------------ query
    def index(self, x, y):
        i = int(round((x - self.ox) / self.res))
        j = int(round((y - self.oy) / self.res))
        return i, j

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def world(self, i, j):
        return self.ox + i * self.res, self.oy + j * self.res

    def clearance_at(self, x, y):
        i, j = self.index(x, y)
        if not self.inside(i, j):
            return 0.0
        return float(self.clearance[j, i])

    def free(self, x, y, clearance_min):
        i, j = self.index(x, y)
        if not self.inside(i, j) or self.occupied[j, i]:
            return False
        return self.clearance[j, i] >= clearance_min

    # ------------------------------------------------------------------- io
    def save(self, path):
        np.savez_compressed(path, occupied=self.occupied, res=self.res,
                            ox=self.ox, oy=self.oy)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        return cls(data['occupied'], float(data['res']), float(data['ox']),
                   float(data['oy']))


# ------------------------------------------------- maze outline / map sanity
def maze_bbox(gmap):
    """Cell bounding box of the occupied cells, i.e. the maze outline.

    Returns ``(i0, i1, j0, j1)`` or ``None`` when the map is empty.
    """
    occupied = gmap.occupied
    if not occupied.any():
        return None
    cols = np.where(occupied.any(axis=0))[0]
    rows = np.where(occupied.any(axis=1))[0]
    return int(cols[0]), int(cols[-1]), int(rows[0]), int(rows[-1])


def restrict_to_maze(gmap, goal, cfg=None):
    """Block everything outside the maze outline (plus a pocket at the goal).

    The saved map only knows "occupied" and "free": cells the robot never saw
    are free as well, so the 0.3 m margin around the maze and every hole in the
    outer wall become a free highway for A*. Restricting the planning domain to
    the bounding box of the wall cells removes that failure mode without
    touching the map itself; a small disc around the goal keeps an exit pad
    outside the outline reachable.

    Returns ``(gmap, info)``; ``info`` carries the numbers the caller logs.
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    info = {'bbox': None, 'leaked': 0, 'blocked': 0, 'empty': False,
            'restricted': bool(cfg['restrict_to_maze'])}
    bb = maze_bbox(gmap)
    if bb is None:
        info['empty'] = True
        return gmap, info
    i0, i1, j0, j1 = bb
    x0, y0 = gmap.world(i0, j0)
    x1, y1 = gmap.world(i1, j1)
    info['bbox'] = (round(x0 - 0.5 * gmap.res, 2), round(x1 + 0.5 * gmap.res, 2),
                    round(y0 - 0.5 * gmap.res, 2), round(y1 + 0.5 * gmap.res, 2))
    inside_box = np.zeros_like(gmap.occupied)
    inside_box[j0:j1 + 1, i0:i1 + 1] = True
    info['leaked'] = int((~gmap.occupied & ~inside_box).sum())
    if not cfg['restrict_to_maze']:
        return gmap, info
    xs = gmap.ox + np.arange(gmap.nx) * gmap.res
    ys = gmap.oy + np.arange(gmap.ny) * gmap.res
    r = float(cfg['goal_pocket'])
    pocket = ((ys - goal[1])[:, None] ** 2 + (xs - goal[0])[None, :] ** 2) <= r * r
    allowed = inside_box | pocket
    blocked = ~allowed & ~gmap.occupied
    info['blocked'] = int(blocked.sum())
    return GridMap(gmap.occupied | blocked, gmap.res, gmap.ox, gmap.oy), info


def check_route(gmap, points, goal, cfg=None):
    """Return ``(on_wall, outside)`` counts for a planned route.

    ``on_wall`` counts points whose cell is occupied; ``outside`` counts points
    that sit outside the maze outline and outside the goal pocket. Both must be
    zero: the first means the route would scrape a wall, the second means it
    would take a shortcut through the 0.3 m strip around the maze.
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    bb = maze_bbox(gmap)
    on_wall = outside = 0
    r = float(cfg['goal_pocket'])
    for x, y in points:
        i, j = gmap.index(x, y)
        if not gmap.inside(i, j) or gmap.occupied[j, i]:
            on_wall += 1
            continue
        if bb is not None:
            i0, i1, j0, j1 = bb
            if not (i0 <= i <= i1 and j0 <= j <= j1):
                if math.hypot(x - goal[0], y - goal[1]) > r:
                    outside += 1
    return on_wall, outside


# --------------------------------------------------------------------- A*
def _nearest_good_cell(gmap, x, y, need, radius=1.2):
    """Closest cell around (x, y) that is at least ``need`` from the walls."""
    i0, j0 = gmap.index(x, y)
    k = int(radius / gmap.res) + 1
    best = None
    best_d = float('inf')
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            i, j = i0 + di, j0 + dj
            if not gmap.inside(i, j) or gmap.occupied[j, i]:
                continue
            d = gmap.clearance[j, i]
            if d < need:
                continue
            dd = math.hypot(di, dj)
            if dd < best_d:
                best_d, best = dd, (i, j)
    return best


def astar(gmap, start, goal, cfg=None):
    """8-connected A* that keeps ``obstacle_radius`` away from the walls.

    The goal itself may sit in a tight spot (e.g. just outside the maze exit)
    -- then the search targets the nearest cell with enough clearance and the
    real goal is appended afterwards.
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    need = cfg['clearance_min'] + cfg['obstacle_radius']
    si, sj = gmap.index(*start)
    gi, gj = gmap.index(*goal)
    if not gmap.inside(si, sj) or not gmap.inside(gi, gj):
        return None
    tail = []
    if gmap.clearance[gj, gi] < need or gmap.occupied[gj, gi]:
        near = _nearest_good_cell(gmap, goal[0], goal[1], need)
        if near is None:
            return None
        gi, gj = near
        tail = [goal]

    n = gmap.nx * gmap.ny
    INF = float('inf')
    g_cost = np.full(n, INF)
    parent = np.full(n, -1, dtype=int)
    sidx = sj * gmap.nx + si
    gidx = gj * gmap.nx + gi
    g_cost[sidx] = 0.0

    def heuristic(i, j):
        return math.hypot(i - gi, j - gj) * gmap.res

    # simple binary heap
    import heapq
    heap = [(heuristic(si, sj), 0.0, sidx)]
    closed = np.zeros(n, dtype=bool)
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while heap:
        _f, g, idx = heapq.heappop(heap)
        if closed[idx]:
            continue
        closed[idx] = True
        if idx == gidx:
            break
        j, i = divmod(idx, gmap.nx)
        for dx, dj, step in moves:
            ni, nj = i + dx, j + dj
            if not gmap.inside(ni, nj):
                continue
            nidx = nj * gmap.nx + ni
            if closed[nidx] or gmap.clearance[nj, ni] < need:
                continue
            ng = g + step * gmap.res
            if ng < g_cost[nidx] - 1e-9:
                g_cost[nidx] = ng
                parent[nidx] = idx
                heapq.heappush(heap, (ng + heuristic(ni, nj), ng, nidx))

    if not np.isfinite(g_cost[gidx]):
        return None
    path = []
    idx = gidx
    while idx != -1:
        j, i = divmod(idx, gmap.nx)
        path.append(gmap.world(i, j))
        idx = parent[idx]
    path.reverse()
    path.extend(tail)
    return path


# -------------------------------------------------------------- path shaping
def _segment_clear(gmap, a, b, clearance_min, step=0.03):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(dist / step) + 1)
    for k in range(n + 1):
        t = k / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if not gmap.free(x, y, clearance_min):
            return False
    return True


def shortcut(gmap, path, cfg=None):
    """Greedy string pulling: jump to the farthest visible waypoint."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    need = cfg['clearance_min'] + cfg['obstacle_radius']
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _segment_clear(gmap, path[i], path[j], need):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _perp_distance(p, a, b):
    if a == b:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / (
        (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (a[0] + (b[0] - a[0]) * t),
                      p[1] - (a[1] + (b[1] - a[1]) * t))


def simplify(path, tol=0.03):
    """Douglas-Peucker."""
    if len(path) < 3:
        return list(path)
    keep = [False] * len(path)
    keep[0] = keep[-1] = True
    stack = [(0, len(path) - 1)]
    while stack:
        i0, i1 = stack.pop()
        best, idx = 0.0, -1
        for k in range(i0 + 1, i1):
            d = _perp_distance(path[k], path[i0], path[i1])
            if d > best:
                best, idx = d, k
        if idx >= 0 and best > tol:
            keep[idx] = True
            stack.append((i0, idx))
            stack.append((idx, i1))
    return [p for p, k in zip(path, keep) if k]


def simplify_safe(gmap, path, cfg=None, tol=0.04):
    """Douglas-Peucker, but never returns a segment that crosses a wall."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    need = cfg['clearance_min'] + cfg['obstacle_radius']
    out = simplify(path, tol=tol)
    ok = all(_segment_clear(gmap, a, b, need) for a, b in zip(out, out[1:]))
    return out if ok else list(path)


def _arc_clear(gmap, pts, need):
    return all(gmap.free(x, y, need) for x, y in pts)


def smooth_path(path, cfg=None):
    """Chaikin subdivision: every vertex is replaced by two points at 1/4 and 3/4.

    Repeated passes converge to a quadratic B-spline, so the polyline becomes
    C1 continuous and its radius of curvature is of the order of the segment
    length (~0.15 m on the diagonal staircase A* produces) instead of the
    single step kink of a raw polyline.  The end points stay where they are.
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    pts = [tuple(p) for p in path]
    if len(pts) < 3:
        return pts
    for _ in range(int(cfg['smooth_iterations'])):
        out = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            out.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            out.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        out.append(pts[-1])
        pts = out
    return pts


def round_corners(gmap, path, cfg=None):
    """Replace each vertex by an arc with the largest radius that fits."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    margin = cfg['corner_margin']
    r_min = cfg['r_min']
    r_max = cfg['r_max']
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    for k in range(1, len(path) - 1):
        a, p, b = path[k - 1], path[k], path[k + 1]
        d1 = math.hypot(p[0] - a[0], p[1] - a[1])
        d2 = math.hypot(b[0] - p[0], b[1] - p[1])
        if d1 < 1e-6 or d2 < 1e-6:
            continue
        clear = gmap.clearance_at(p[0], p[1]) - margin
        r = min(r_max, 0.5 * d1, 0.5 * d2, max(clear, 0.0))
        if r < r_min:
            out.append(p)
            continue
        # tangent length and arc centre (see the geometry of a fillet)
        ux, uy = (p[0] - a[0]) / d1, (p[1] - a[1]) / d1
        vx, vy = (b[0] - p[0]) / d2, (b[1] - p[1]) / d2
        cross = ux * vy - uy * vx
        dot = max(-1.0, min(1.0, ux * vx + uy * vy))
        turn = math.acos(dot)               # 0 = straight, pi/2 = right angle
        if turn < 1e-3:
            out.append(p)
            continue
        if cross > 0:
            nx, ny = -uy, ux                # centre is to the left
        else:
            nx, ny = uy, -ux                # centre is to the right
        want = max(cfg['clearance_min'], cfg['arc_clearance'])
        arc = None
        for scale in (1.0, 0.7, 0.5, 0.35, 0.25):
            rr = r * scale
            if rr < r_min * 0.5:
                break
            tang = rr * math.tan(turn / 2.0)
            if tang > d1 or tang > d2:
                continue
            t1 = (p[0] - ux * tang, p[1] - uy * tang)
            t2 = (p[0] + vx * tang, p[1] + vy * tang)
            cx = t1[0] + nx * rr
            cy = t1[1] + ny * rr
            ang1 = math.atan2(t1[1] - cy, t1[0] - cx)
            ang2 = math.atan2(t2[1] - cy, t2[0] - cx)
            d_ang = math.atan2(math.sin(ang2 - ang1), math.cos(ang2 - ang1))
            if cross > 0 and d_ang < 0.0:
                d_ang += 2.0 * math.pi
            elif cross < 0 and d_ang > 0.0:
                d_ang -= 2.0 * math.pi
            steps = max(4, int(abs(d_ang) * rr / cfg['sample_step']) + 2)
            cand = []
            for kk in range(steps + 1):
                ang = ang1 + d_ang * kk / steps
                cand.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
            if _arc_clear(gmap, cand, want):
                arc = cand
                break
        if arc is None:
            out.append(p)
        else:
            out.extend(arc)
    out.append(path[-1])
    return out


def resample(path, step=None, cfg=None):
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    step = step or cfg['sample_step']
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(seg / step))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def curvature(points):
    """Discrete curvature at every point (m^-1), from the circumscribed circle."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    k = np.zeros(n)
    if n < 3:
        return k
    for i in range(1, n - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        ab = np.linalg.norm(b - a)
        bc = np.linalg.norm(c - b)
        ca = np.linalg.norm(a - c)
        if ab < 1e-9 or bc < 1e-9 or ca < 1e-9:
            continue
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if area2 < 1e-12:
            continue
        k[i] = 2.0 * area2 / (ab * bc * ca)
    k[0], k[-1] = k[1] if n > 1 else 0.0, k[-2] if n > 1 else 0.0
    return k


def effective_curvature(points, cfg=None):
    """Curvature used by both the speed profile and the tracker.

    The raw three point estimate is ill conditioned on a 0.02 m sampling: a
    couple of millimetres of jitter turns into a spike of 10 rad/m, which would
    make the profile stutter. Both the planner and ``pid_tracker`` therefore
    average it over a small window (``curv_smooth``) before using it.
    """
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    k = np.abs(curvature(points))
    w = int(cfg.get('curv_smooth', 5))
    if w < 3 or len(k) <= w:
        return k
    kernel = np.ones(w) / float(w)
    return np.convolve(np.pad(k, (w // 2, w // 2), mode='edge'), kernel,
                       mode='valid')[:len(k)]


def speed_profile(points, cfg=None):
    """Time-optimal speed per point under curvature and acceleration limits."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < 2:
        return np.zeros(n), 0.0
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    curv = effective_curvature(points, cfg)
    with np.errstate(divide='ignore', invalid='ignore'):
        v_curve = np.where(curv > 1e-6, np.sqrt(cfg['a_lat'] / np.maximum(curv, 1e-6)),
                           np.inf)
        # A tight corner is limited by how fast the wheels can yaw, not only by
        # the lateral acceleration: v * kappa <= max_yaw_rate.  Without this the
        # profile would ask for 4-5 rad/s at a 0.03 m fillet, the tracker
        # saturates at 2.0 rad/s and the car drives straight into the corner.
        v_yaw = np.where(curv > 1e-6, cfg['max_yaw_rate'] / np.maximum(curv, 1e-6),
                         np.inf)
    v = np.minimum(cfg['v_max'], np.minimum(v_curve, v_yaw))
    # floor keeps the controller alive, but never above the yaw rate cap
    v = np.maximum(v, np.minimum(0.05, v_yaw))

    # forward: acceleration limited
    for i in range(1, n):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * cfg['a_acc'] * ds[i - 1]))
    # backward: braking limited
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * cfg['a_dec'] * ds[i]))

    v_seg = np.minimum(v[:-1], v[1:])
    v_seg = np.maximum(v_seg, 0.02)
    total = float(np.sum(ds / v_seg))
    return v, total


def smoothing_pass(gmap, path, cfg=None):
    """Smooth the polyline, backing off if the curve would clip a wall."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    need = max(0.05, float(cfg['clearance_min']))
    for iters in range(int(cfg['smooth_iterations']), 0, -1):
        cand = smooth_path(path, dict(cfg, smooth_iterations=iters))
        if all(_segment_clear(gmap, a, b, need) for a, b in zip(cand, cand[1:])):
            return cand
    return list(path)


def plan_race(gmap, start, goal, cfg=None):
    """Full pipeline: grid -> polyline -> arcs -> speed profile."""
    cfg = dict(DEFAULT_CFG, **(cfg or {}))
    gmap, info = restrict_to_maze(gmap, goal, cfg)
    if info['empty']:
        return None
    raw = astar(gmap, start, goal, cfg)
    if raw is None:
        return None
    path = simplify(shortcut(gmap, raw, cfg), tol=cfg['simplify_tol'])
    path = smoothing_pass(gmap, path, cfg)
    path = resample(path, cfg=cfg)
    speeds, est_time = speed_profile(path, cfg)
    return {
        'points': path,
        'speeds': speeds,
        'time': est_time,
        'length': float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                            for a, b in zip(path, path[1:]))),
        'raw_points': raw,
        'map_info': info,
    }
