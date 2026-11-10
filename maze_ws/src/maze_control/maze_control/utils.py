"""Small geometry / LaserScan helpers shared by all control nodes."""

import math


def yaw_from_quaternion(q):
    """Extract the yaw angle (rad) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def pose_from_odom(msg):
    """(x, y, yaw) from a nav_msgs/Odometry message."""
    p = msg.pose.pose.position
    return p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)


def sector_min(scan, angle_low, angle_high):
    """Minimum finite range (m) inside [angle_low, angle_high] radians.

    Angles are measured in the laser frame (0 = straight ahead, positive =
    counter-clockwise / left).  ``inf`` is returned when the sector is empty.
    """
    best = float('inf')
    angle = scan.angle_min
    inc = scan.angle_increment
    for r in scan.ranges:
        if math.isfinite(r) and r > 0.0:
            a = normalize_angle(angle)
            if angle_low <= a <= angle_high and r < best:
                best = r
        angle += inc
    return best


def scan_sectors(scan):
    """Return the six sector distances used by the reactive controllers.

    Keys: front, front_center, front_left, front_right, left, right.  The
    sectors intentionally overlap a little so that no narrow wall corner can
    slip between them.  An empty sector is reported as 10.0 m (far away)
    instead of ``inf`` so that arithmetic between sectors stays finite.

    ``front`` is the wide +-20 deg fan; it is what the speed ramp and the
    corner geometry use, because a wall 20 deg off the heading still means
    "mind the gap".  ``front_center`` is the narrow +-6 deg cone and is what
    the "is the way ahead blocked?" test uses: in a 0.7 m corridor a robot
    that hugs one wall (0.2 m off centre, a few degrees of heading error) has
    that side wall inside the wide fan, which used to read as a wall dead
    ahead and made the follower announce a dead end in an open corridor.
    """
    raw = {
        'front': sector_min(scan, -0.35, 0.35),          # +-20 deg
        'front_center': sector_min(scan, -0.105, 0.105),  # +-6 deg
        'front_left': sector_min(scan, 0.35, 1.05),      # 20 .. 60 deg
        'front_right': sector_min(scan, -1.05, -0.35),   # -60 .. -20 deg
        'left': sector_min(scan, 1.05, 1.83),            # 60 .. 105 deg
        'right': sector_min(scan, -1.83, -1.05),         # -105 .. -60 deg
    }
    return {k: (min(v, 10.0) if math.isfinite(v) else 10.0)
            for k, v in raw.items()}
