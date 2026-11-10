"""Record the driven trajectory as a nav_msgs/Path (for RViz).

Publishes /actual_path with one pose per odometry message (throttled).
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')

        self.declare_parameter('min_distance', 0.01)   # m between stored poses
        self.declare_parameter('max_poses', 20000)
        self.declare_parameter('publish_period', 0.2)  # s
        self.declare_parameter('jump_distance', 0.5)   # m; teleport -> new trail

        self.min_distance = float(self.get_parameter('min_distance').value)
        self.jump_distance = float(self.get_parameter('jump_distance').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.period = float(self.get_parameter('publish_period').value)

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_pose = None
        self.pub = self.create_publisher(Path, 'actual_path', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(self.period, self._publish)
        self.get_logger().info('publishing odometry path on /actual_path')

        # A stale node left over from an earlier run keeps its own history and
        # both publishers take turns on /actual_path, which is what makes RViz
        # look like scribble.  Say it loudly instead of debugging it later.
        others = self.count_publishers('actual_path') - 1
        if others > 0:
            self.get_logger().error(
                'another odom_path_publisher is already running (%d extra '
                'publisher(s) on /actual_path): the RViz trail will look like '
                'scribble. Run tools/stop_all.sh before launching.' % others)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_pose is not None:
            d = ((p.x - self.last_pose.x) ** 2 + (p.y - self.last_pose.y) ** 2) ** 0.5
            if d > self.jump_distance:
                # odometry jumped (restarted simulation): start a fresh trail
                # instead of drawing a straight line across the map
                self.get_logger().warn(
                    'odometry jumped %.2f m: clearing the trail' % d)
                self.path.poses = []
            elif d < self.min_distance:
                return
        self.last_pose = p
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            del self.path.poses[:len(self.path.poses) - self.max_poses]

    def _publish(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
