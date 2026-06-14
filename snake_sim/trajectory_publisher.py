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

        # Rate (Hz) at which the "desired position" carrot marker is published.
        self.declare_parameter('marker_rate', 30.0)

        # Path-following carrot: the desired point is the robot CoM projected
        # onto the path, advanced this many metres along the path (arc length).
        # Past the final vertex it extrapolates along the last segment heading.
        self.declare_parameter('lookahead_distance', 0.3)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traj_pub = self.create_publisher(
            MultiDOFJointTrajectory, '/snake/desired_trajectory', latched_qos)
        self.path_pub = self.create_publisher(
            Path, '/snake/desired_path', latched_qos)
        self.desired_marker_pub = self.create_publisher(
            Marker, '/snake/desired_position', 10)

        # Latest robot CoM (from center_of_mass_calculator), used to place the
        # path-following carrot.
        self._latest_com = None
        self.create_subscription(
            Marker, '/snake/center_of_mass', self._on_com, 10)

        points = self._build_points()
        traj = self._make_trajectory(points)
        self.traj_pub.publish(traj)
        self.path_pub.publish(self._make_path(points))

        # Cache the path geometry + cumulative arc length for the carrot lookup.
        self._traj_points = points
        self._traj_cum = self._cumulative_arclength(points)

        times = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                 for pt in traj.points]
        duration_s = times[-1] if times else 0.0
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

    def _on_com(self, msg):
        self._latest_com = (msg.pose.position.x, msg.pose.position.y)

    @staticmethod
    def _cumulative_arclength(points):
        cum = [0.0]
        for i in range(1, len(points)):
            cum.append(cum[-1] + math.hypot(points[i][0] - points[i - 1][0],
                                            points[i][1] - points[i - 1][1]))
        return cum

    def _carrot_xy(self):
        """Desired point for path following: project the CoM onto the path,
        then advance `lookahead_distance` along it (arc length).

        Falls back to the path start until a CoM has been received.
        """
        points = self._traj_points
        com = self._latest_com
        if com is None or len(points) < 2:
            return points[0] if points else (0.0, 0.0)

        cum = self._traj_cum
        # Nearest projection of the CoM onto the polyline -> its arc length.
        best_d2 = None
        s_closest = 0.0
        for i in range(len(points) - 1):
            ax, ay = points[i]
            bx, by = points[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 <= 1e-12:
                t, px, py = 0.0, ax, ay
            else:
                t = ((com[0] - ax) * dx + (com[1] - ay) * dy) / seg2
                t = max(0.0, min(1.0, t))
                px, py = ax + t * dx, ay + t * dy
            d2 = (com[0] - px) ** 2 + (com[1] - py) ** 2
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                s_closest = cum[i] + t * math.sqrt(seg2)

        return self._point_at_arclength(s_closest + self.lookahead_distance)

    def _point_at_arclength(self, s):
        points = self._traj_points
        cum = self._traj_cum
        total = cum[-1]
        if s <= 0.0:
            return points[0]
        if s >= total:
            # Extrapolate beyond the final vertex along the last segment.
            ax, ay = points[-2]
            bx, by = points[-1]
            seg = math.hypot(bx - ax, by - ay)
            if seg <= 1e-12:
                return points[-1]
            over = s - total
            return (bx + over * (bx - ax) / seg, by + over * (by - ay) / seg)
        # Linear scan is fine for ~hundreds of points.
        for i in range(len(cum) - 1):
            if cum[i] <= s <= cum[i + 1]:
                seg = cum[i + 1] - cum[i]
                t = (s - cum[i]) / seg if seg > 1e-12 else 0.0
                ax, ay = points[i]
                bx, by = points[i + 1]
                return (ax + t * (bx - ax), ay + t * (by - ay))
        return points[-1]

    def _publish_desired_marker(self):
        x, y = self._carrot_xy()

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
