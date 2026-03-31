import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Trajectory type: 'waypoints', 'line', 'circle', 'sine'
        self.declare_parameter('type', 'waypoints')

        # Waypoints mode: list of [x, y] pairs flattened to [x1, y1, x2, y2, ...]
        self.declare_parameter('waypoints', [0.0, 0.0, 1.0, 0.0])

        # Parametric shape parameters
        self.declare_parameter('radius', 0.5)          # circle radius
        self.declare_parameter('amplitude', 0.3)        # sine amplitude
        self.declare_parameter('wavelength', 1.0)        # sine wavelength
        self.declare_parameter('length', 2.0)            # line/sine length
        self.declare_parameter('num_points', 100)        # interpolation resolution
        self.declare_parameter('origin_x', 0.0)          # shape origin X
        self.declare_parameter('origin_y', 0.0)          # shape origin Y

        # Publish with transient local durability so RViz gets it even if it starts later
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/snake/desired_trajectory', latched_qos)

        path = self._build_path()
        self.path_pub.publish(path)
        self.get_logger().info(
            f'Published desired trajectory: type={self.get_parameter("type").value}, '
            f'{len(path.poses)} points'
        )

    def _build_path(self):
        traj_type = self.get_parameter('type').value
        if traj_type == 'waypoints':
            return self._build_waypoints()
        elif traj_type == 'line':
            return self._build_line()
        elif traj_type == 'circle':
            return self._build_circle()
        elif traj_type == 'sine':
            return self._build_sine()
        else:
            self.get_logger().error(f'Unknown trajectory type: {traj_type}')
            return Path()

    def _make_path(self, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _build_waypoints(self):
        flat = self.get_parameter('waypoints').value
        points = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        return self._make_path(points)

    def _build_line(self):
        length = self.get_parameter('length').value
        n = self.get_parameter('num_points').value
        ox = self.get_parameter('origin_x').value
        oy = self.get_parameter('origin_y').value
        points = [(ox + i * length / (n - 1), oy) for i in range(n)]
        return self._make_path(points)

    def _build_circle(self):
        r = self.get_parameter('radius').value
        n = self.get_parameter('num_points').value
        ox = self.get_parameter('origin_x').value
        oy = self.get_parameter('origin_y').value
        points = [
            (ox + r * math.cos(2 * math.pi * i / n),
             oy + r * math.sin(2 * math.pi * i / n))
            for i in range(n + 1)  # +1 to close the loop
        ]
        return self._make_path(points)

    def _build_sine(self):
        length = self.get_parameter('length').value
        amp = self.get_parameter('amplitude').value
        wl = self.get_parameter('wavelength').value
        n = self.get_parameter('num_points').value
        ox = self.get_parameter('origin_x').value
        oy = self.get_parameter('origin_y').value
        points = [
            (ox + i * length / (n - 1),
             oy + amp * math.sin(2 * math.pi * (i * length / (n - 1)) / wl))
            for i in range(n)
        ]
        return self._make_path(points)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
