#!/usr/bin/env python3
"""Second pass: plan the fastest route on the saved map and drive it.

The node loads the map written by ``map_builder`` (first pass), runs the A*
+ corner-arc planner once, then tracks the resulting polyline with pure
pursuit while respecting the curvature limited speed profile.
"""

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from maze_control.control_law import DEFAULT_PP_CFG, pure_pursuit_command
from maze_control.pid_tracker import DEFAULT_PID_CFG, PidPathFollower
from maze_control.race_planner import (GridMap, check_route, plan_race,
                                       restrict_to_maze)
from maze_control.utils import clamp, pose_from_odom


class PathRacer(Node):
    def __init__(self):
        super().__init__('path_racer')
        self.declare_parameter('map_file', os.path.expanduser('~/maze_map.npz'))
        self.declare_parameter('goal_x', 3.60)
        self.declare_parameter('goal_y', 4.15)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('max_linear', DEFAULT_PP_CFG['max_linear'])
        self.declare_parameter('lookahead', DEFAULT_PP_CFG['lookahead'])
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('plan_delay', 1.5)
        self.declare_parameter('controller', 'pid')
        self.declare_parameter('obstacle_radius', 0.18)
        self.declare_parameter('kp_lat', DEFAULT_PID_CFG['kp_lat'])
        self.declare_parameter('ki_lat', DEFAULT_PID_CFG['ki_lat'])
        self.declare_parameter('kd_lat', DEFAULT_PID_CFG['kd_lat'])
        self.declare_parameter('kp_theta', DEFAULT_PID_CFG['kp_theta'])
        self.declare_parameter('timeout', 120.0)

        self.map_file = str(self.get_parameter('map_file').value)
        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))
        self.scale = float(self.get_parameter('speed_scale').value)
        self.cfg = dict(DEFAULT_PP_CFG)
        self.cfg['max_linear'] = float(self.get_parameter('max_linear').value)
        self.cfg['lookahead'] = float(self.get_parameter('lookahead').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.plan_delay = float(self.get_parameter('plan_delay').value)
        self.controller = str(self.get_parameter('controller').value)
        self.plan_cfg = {'obstacle_radius':
                         float(self.get_parameter('obstacle_radius').value)}
        self.pid_cfg = dict(DEFAULT_PID_CFG)
        for key in ('kp_lat', 'ki_lat', 'kd_lat', 'kp_theta'):
            self.pid_cfg[key] = float(self.get_parameter(key).value)
        self.pid_cfg['max_linear'] = self.cfg['max_linear']
        self.follower = None

        self.pose = None
        self.path = None
        self.speeds = None
        self.index = 0
        self.done = False
        self.t0 = None
        self.plan_tried = False
        self.last_move_pose = None
        self.last_move_time = 0.0
        self.recover_until = 0.0

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, 'reference_path', 1)
        self.pub_state = self.create_publisher(String, 'race_state', 10)
        self.pub_done = self.create_publisher(Bool, 'race_done', 10)
        self.pub_err = self.create_publisher(Float32, 'cross_track_error', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self.tick)

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def plan(self):
        self.plan_tried = True
        if not os.path.exists(self.map_file):
            self.get_logger().error('map file %s not found, run the explore '
                                    'pass first' % self.map_file)
            return False
        gmap = GridMap.load(self.map_file)
        gmap, info = restrict_to_maze(gmap, self.goal, self.plan_cfg)
        if info['empty']:
            self.get_logger().error('saved map is empty, run the explore pass '
                                    'first')
            return False
        self.get_logger().info(
            'map: outline x=%.2f..%.2f y=%.2f..%.2f, %d cells blocked to keep '
            'the route inside the maze'
            % (info['bbox'] + (info['blocked'],)))
        result = plan_race(gmap, self.pose[:2], self.goal, self.plan_cfg)
        if result is None:
            self.get_logger().error(
                'no route found on the saved map -- the explore pass probably '
                'missed part of the maze, run it again')
            return False
        on_wall, outside = check_route(gmap, result['points'], self.goal,
                                       self.plan_cfg)
        if on_wall or outside:
            self.get_logger().error(
                'planned route rejected: %d points on a wall, %d points outside '
                'the maze -- re-run the explore pass' % (on_wall, outside))
            return False
        self.path = result['points']
        self.speeds = result['speeds']
        self.get_logger().info(
            'route: %d points, %.2f m, estimated %.1f s'
            % (len(self.path), result['length'], result['time']))
        self.publish_path()
        self.follower = PidPathFollower(self.path, result['speeds'], self.pid_cfg)
        self.get_logger().info('tracker: %s' % self.controller)
        return True

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def tick(self):
        if self.done:
            self.stop()
            return
        if self.pose is None:
            return
        if not self.plan_tried:
            if self.t0 is None:
                self.t0 = self.get_clock().now().nanoseconds * 1e-9
            if self.get_clock().now().nanoseconds * 1e-9 - self.t0 < self.plan_delay:
                return
            self.get_logger().info('planning on the saved map from (%.2f, %.2f)'
                                   % self.pose[:2])
            if not self.plan():
                self.done = True
                self.stop()
                return
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
        x, y, yaw = self.pose
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        if elapsed > self.timeout:
            self.get_logger().error('race timeout after %.1f s' % elapsed)
            self.done = True
            self.stop()
            return
        if self.controller == 'pid' and self.follower is not None:
            v, w, state = self.follower.update(x, y, yaw, dt=1.0 / 20.0)
            remaining = self.follower.geom['length'] - state['s']
            index = self.follower.hint
            alpha = state['e']
            self.index = index
            if self.follower.finished:
                self.get_logger().info('RACE COMPLETE at (%.2f, %.2f) in %.1f s'
                                       % (x, y, elapsed))
                self.done = True
                done = Bool()
                done.data = True
                self.pub_done.publish(done)
                self.stop()
                return
        else:
            v, w, carrot, index, remaining, alpha = pure_pursuit_command(
                x, y, yaw, self.path, self.cfg, self.index)
            self.index = index
        if self.speeds is not None and index < len(self.speeds):
            limit = self.speeds[index] * self.scale
            v = min(v, limit)
        # slow down for the sharp bits of the profile
        if remaining < 0.20:
            v = min(v, 0.05 + 0.5 * remaining)
        # stuck watchdog: wheels spin but the robot does not move
        now = self.get_clock().now().nanoseconds * 1e-9
        if elapsed < self.recover_until:
            self.publish_cmd(-0.06, 0.6)
            return
        if self.last_move_pose is None or math.hypot(
                x - self.last_move_pose[0], y - self.last_move_pose[1]) > 0.02:
            self.last_move_pose = (x, y)
            self.last_move_time = now
        elif now - self.last_move_time > 2.0 and v > 0.05:
            self.get_logger().warn('stuck at (%.2f, %.2f), backing off' % (x, y))
            self.recover_until = elapsed + 1.2
            self.last_move_pose = None
            self.publish_cmd(-0.06, 0.6)
            return
        self.publish_cmd(v, w)
        err = Float32()
        err.data = float(alpha)
        self.pub_err.publish(err)
        state = String()
        state.data = 'RUN t=%.1f idx=%d rem=%.2f v=%.2f w=%.2f' % (
            elapsed, index, remaining, v, w)
        self.pub_state.publish(state)
        if remaining < self.cfg['goal_tolerance']:
            self.get_logger().info(
                'RACE COMPLETE at (%.2f, %.2f) in %.1f s'
                % (x, y, elapsed))
            self.done = True
            done = Bool()
            done.data = True
            self.pub_done.publish(done)
            self.stop()

    def publish_cmd(self, v, w):
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(clamp(w, -2.0, 2.0))
        self.pub_cmd.publish(twist)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PathRacer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
