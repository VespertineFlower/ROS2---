#!/usr/bin/env python3
"""First pass: drive around with the left-wall follower and build a map.

The node fuses ``/scan`` into an occupancy grid expressed in the ``odom``
frame.  Points that the laser hits are marked as occupied, the clearance
field is derived from them with a distance transform, and the resulting map
is published on ``/map`` (OccupancyGrid) and written to ``map_file`` (.npz)
when the node shuts down -- so the second pass can plan on it.
"""

import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_control.race_planner import GridMap
from maze_control.utils import pose_from_odom


class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('x_min', -0.30)
        self.declare_parameter('x_max', 4.30)
        self.declare_parameter('y_min', -0.30)
        self.declare_parameter('y_max', 4.30)
        self.declare_parameter('map_file', os.path.expanduser('~/maze_map.npz'))
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('dilate_cells', 0)

        res = float(self.get_parameter('resolution').value)
        xmin = float(self.get_parameter('x_min').value)
        ymin = float(self.get_parameter('y_min').value)
        xmax = float(self.get_parameter('x_max').value)
        ymax = float(self.get_parameter('y_max').value)
        self.map_file = str(self.get_parameter('map_file').value)
        self.dilate = int(self.get_parameter('dilate_cells').value)

        self.raw = np.zeros((int((ymax - ymin) / res) + 1,
                             int((xmax - xmin) / res) + 1), dtype=bool)
        self.occ = self.raw
        self.gmap = GridMap(self.occ, res, xmin, ymin)
        self.pose = None
        self.hits = 0

        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub_map = self.create_publisher(OccupancyGrid, 'map', 1)
        self.pub_state = self.create_publisher(String, 'map_builder_state', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 0.2), self.publish_map)
        self.get_logger().info(
            'map_builder: grid %dx%d @ %.2f m -> %s'
            % (self.occ.shape[0], self.occ.shape[1], res, self.map_file))

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def on_scan(self, msg):
        if self.pose is None:
            return
        x0, y0, yaw = self.pose
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        ang = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        rng = np.asarray(msg.ranges, dtype=float)
        keep = np.isfinite(rng) & (rng > msg.range_min) & (rng < msg.range_max)
        if not keep.any():
            return
        lx = rng[keep] * np.cos(ang[keep])
        ly = rng[keep] * np.sin(ang[keep])
        wx = x0 + cos_y * lx - sin_y * ly
        wy = y0 + sin_y * lx + cos_y * ly
        r = self.gmap.res
        ii = np.round((wx - self.gmap.ox) / r).astype(int)
        jj = np.round((wy - self.gmap.oy) / r).astype(int)
        ok = (ii >= 0) & (ii < self.occ.shape[1]) & (jj >= 0) & (jj < self.occ.shape[0])
        self.raw[jj[ok], ii[ok]] = True
        occ = self.raw
        for _ in range(max(self.dilate, 0)):
            grown = occ.copy()
            grown[1:, :] |= occ[:-1, :]
            grown[:-1, :] |= occ[1:, :]
            grown[:, 1:] |= occ[:, :-1]
            grown[:, :-1] |= occ[:, 1:]
            occ = grown
        self.occ = occ
        self.gmap.occupied = self.occ
        self.hits += 1

    def publish_map(self):
        self.gmap.occupied = self.occ
        self.gmap._clearance()
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.gmap.res
        grid.info.width = self.occ.shape[1]
        grid.info.height = self.occ.shape[0]
        grid.info.origin.position.x = self.gmap.ox
        grid.info.origin.position.y = self.gmap.oy
        grid.data = [100 if v else 0 for v in self.occ.ravel().tolist()]
        self.pub_map.publish(grid)
        state = String()
        state.data = 'cells=%d scans=%d pose=%s' % (
            int(self.occ.sum()), self.hits,
            'None' if self.pose is None else '(%.2f, %.2f)' % self.pose[:2])
        self.pub_state.publish(state)

    def save(self):
        if self.hits == 0:
            self.get_logger().warn('map_builder: no scan data, nothing saved')
            return
        self.gmap.occupied = self.occ
        self.gmap._clearance()
        self.gmap.save(self.map_file)
        self.get_logger().info('map saved to %s (%d occupied cells)'
                               % (self.map_file, int(self.occ.sum())))


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
