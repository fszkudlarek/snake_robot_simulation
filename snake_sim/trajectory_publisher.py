import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Trajectory type: 'waypoints', 'line', 'circle', 'sine'
        self.declare_parameter('type', 'waypoints')

        # Waypoints mode: list of [x, y] pairs flattened to [x1, y1, x2, y2, ...]
        self.declare_parameter('waypoints', [0.0, 0.0, 1.0, 0.0])

        # Parametric shape parameters
        self.declare_parameter('radius', 0.5)
        self.declare_parameter('amplitude', 0.3)
        self.declare_parameter('wavelength', 1.0)
        self.declare_parameter('length', 2.0)
        self.declare_parameter('num_points', 100)
        # Start point shared by all parametric shapes — the robot's initial COM.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.36)
        # Line heading in degrees CCW from +X (0 -> straight along +X).
        self.declare_parameter('angle', 0.0)
        # Circle direction: 'ccw' curves toward +Y after the initial +X,
        # 'cw' toward -Y.
        self.declare_parameter('direction', 'ccw')

        # Time parametrization: speed (m/s) along the curve. Each point's
        # time_from_start = cumulative_arc_length(point) / linear_speed.
        self.declare_parameter('linear_speed', 0.1)

        # Rate (Hz) at which the moving "desired position" marker is published.
        self.declare_parameter('marker_rate', 30.0)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traj_pub = self.create_publisher(
            MultiDOFJointTrajectory, '/snake/desired_trajectory', latched_qos)
        self.path_pub = self.create_publisher(
            Path, '/snake/desired_path', latched_qos)
        self.desired_marker_pub = self.create_publisher(
            Marker, '/snake/desired_position', 10)

        points = self._build_points()
        traj = self._make_trajectory(points)
        self.traj_pub.publish(traj)
        self.path_pub.publish(self._make_path(points))

        # Cache (x, y, time_from_start_seconds) for marker interpolation.
        self._traj_points = points
        self._traj_times = [
            pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            for pt in traj.points
        ]
        self._traj_start = self.get_clock().now()

        duration_s = self._traj_times[-1] if self._traj_times else 0.0
        self.get_logger().info(
            f'Published desired trajectory: type={self.get_parameter("type").value}, '
            f'{len(points)} points, duration={duration_s:.2f}s '
            f'@ {self.get_parameter("linear_speed").value} m/s'
        )

        marker_rate = self.get_parameter('marker_rate').value
        if marker_rate > 0 and self._traj_points:
            self.create_timer(1.0 / marker_rate, self._publish_desired_marker)

    def _build_points(self):
        traj_type = self.get_parameter('type').value
        if traj_type == 'waypoints':
            flat = self.get_parameter('waypoints').value
            return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        elif traj_type == 'line':
            return self._build_line()
        elif traj_type == 'circle':
            return self._build_circle()
        elif traj_type == 'sine':
            return self._build_sine()
        else:
            self.get_logger().error(f'Unknown trajectory type: {traj_type}')
            return []

    def _build_line(self):
        length = self.get_parameter('length').value
        angle = math.radians(self.get_parameter('angle').value)
        n = self.get_parameter('num_points').value
        sx = self.get_parameter('start_x').value
        sy = self.get_parameter('start_y').value
        # Straight segment of `length` from the start point, heading `angle`
        # degrees CCW from +X (angle=0 -> +X, angle=90 -> +Y).
        dx, dy = math.cos(angle), math.sin(angle)
        return [
            (sx + (i * length / (n - 1)) * dx,
             sy + (i * length / (n - 1)) * dy)
            for i in range(n)
        ]

    def _build_circle(self):
        r = self.get_parameter('radius').value
        n = self.get_parameter('num_points').value
        sx = self.get_parameter('start_x').value
        sy = self.get_parameter('start_y').value
        d = str(self.get_parameter('direction').value).strip().lower()
        if d in ('cw', 'clockwise', 'right'):
            s = -1  # curve toward -Y after the initial +X
        elif d in ('ccw', 'counterclockwise', 'counter-clockwise', 'left'):
            s = 1   # curve toward +Y
        else:
            self.get_logger().warn(f"Unknown circle direction '{d}'; using 'ccw'.")
            s = 1
        # Circle tangent to +X at the start point: the centre sits r metres
        # along s*Y from the start, so the path leaves heading +X and then
        # curves toward s*Y.
        cy = sy + s * r
        return [
            (sx + r * math.cos(s * (2 * math.pi * i / n - math.pi / 2)),
             cy + r * math.sin(s * (2 * math.pi * i / n - math.pi / 2)))
            for i in range(n + 1)  # +1 to close the loop
        ]

    def _build_sine(self):
        length = self.get_parameter('length').value
        amp = self.get_parameter('amplitude').value
        wl = self.get_parameter('wavelength').value
        n = self.get_parameter('num_points').value
        sx = self.get_parameter('start_x').value
        sy = self.get_parameter('start_y').value
        # Travels along +X from the start point, oscillating `amp` in Y.
        return [
            (sx + i * length / (n - 1),
             sy + amp * math.sin(2 * math.pi * (i * length / (n - 1)) / wl))
            for i in range(n)
        ]

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

    def _make_trajectory(self, points):
        speed = self.get_parameter('linear_speed').value
        if speed <= 0.0:
            self.get_logger().error(
                f'linear_speed must be > 0, got {speed}; trajectory will have zero timing.'
            )

        traj = MultiDOFJointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'odom'
        traj.joint_names = ['base_link']

        cumulative = 0.0
        for i, (x, y) in enumerate(points):
            if i > 0:
                dx = x - points[i - 1][0]
                dy = y - points[i - 1][1]
                cumulative += math.hypot(dx, dy)

            pt = MultiDOFJointTrajectoryPoint()
            tf = Transform()
            tf.translation.x = x
            tf.translation.y = y
            tf.translation.z = 0.0
            tf.rotation.w = 1.0
            pt.transforms.append(tf)

            t = cumulative / speed if speed > 0 else 0.0
            pt.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(pt)

        return traj


    def _desired_xy_at(self, elapsed):
        times = self._traj_times
        points = self._traj_points
        if elapsed <= times[0]:
            return points[0]
        if elapsed >= times[-1]:
            return points[-1]
        # Linear scan is fine for ~hundreds of points.
        for i in range(len(times) - 1):
            if times[i] <= elapsed <= times[i + 1]:
                t0, t1 = times[i], times[i + 1]
                p0, p1 = points[i], points[i + 1]
                alpha = (elapsed - t0) / (t1 - t0) if t1 > t0 else 0.0
                return (p0[0] + alpha * (p1[0] - p0[0]),
                        p0[1] + alpha * (p1[1] - p0[1]))
        return points[-1]

    def _publish_desired_marker(self):
        elapsed = (self.get_clock().now() - self._traj_start).nanoseconds * 1e-9
        x, y = self._desired_xy_at(elapsed)

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.ns = 'desired_position'
        msg.id = 0
        msg.type = Marker.CYLINDER
        msg.action = Marker.ADD
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.25
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.01
        msg.scale.y = 0.01
        msg.scale.z = 0.5
        msg.color.r = 0.0
        msg.color.g = 0.4
        msg.color.b = 1.0
        msg.color.a = 0.8
        self.desired_marker_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
