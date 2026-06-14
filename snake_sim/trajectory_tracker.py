import math
import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


# Whole-body link set used for the PCA principal-axis fit. Same set the
# offline metrics use (see scripts/compute_avg_com.py and the body logger).
# The first frame is treated as the head end for PCA sign disambiguation.
BODY_LINKS = (
    ['motor_with_onshape_mounting']
    + [f'motor_with_onshape_mounting_{i}' for i in range(2, 7)]
    + ['motor_with_no_arms']
)
HEAD_LINK = BODY_LINKS[0]


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pca_yaw_2d(points: list[tuple[float, float]],
               head_xy: tuple[float, float]) -> float | None:
    """Body yaw from 2x2 PCA closed form (eq. used in compute_avg_com.py):
        theta = 0.5 * atan2(2*Sxy, Sxx - Syy)
    The principal axis is sign-ambiguous; we flip it if needed so that the
    head end sits on the positive side, i.e. theta points from the body
    centroid toward the head.

    Returns None if fewer than two points are provided.
    """
    if len(points) < 2:
        return None

    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)

    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)

    # Sign disambiguation: flip theta by pi if the axis points away from
    # the head end.
    axis_x, axis_y = math.cos(theta), math.sin(theta)
    head_dx, head_dy = head_xy[0] - mx, head_xy[1] - my
    if axis_x * head_dx + axis_y * head_dy < 0.0:
        theta = wrap_to_pi(theta + math.pi)
    return theta


class TrajectoryTracker(Node):
    """Closed-loop steering for the sidewinding controller.

    Subscribes to the path-following desired point (the carrot published by
    trajectory_publisher on /snake/desired_position), to the mass-weighted
    body CoM marker, and to TF (for whole-body link positions). At a fixed
    rate:
      * reads the current desired (carrot) point;
      * computes the robot's *body* heading via the same PCA principal-axis
        fit the offline metrics use, sign-disambiguated toward the head;
      * publishes δ = clip(K_p · heading_error, ±delta_max) on
        /movement_controller/delta. The sidewinding controller consumes δ
        via eq. 3.26 and clips it again to its own delta_max.
    """

    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('K_p', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        # Local saturation on |δ| so a runaway error can't push wild values
        # onto the topic. The controller clips again to its own delta_max.
        self.declare_parameter('delta_max', 15.0 * math.pi / 180.0)
        # In sidewinding the body's PCA principal axis is NOT the direction
        # of travel — the robot drifts along an axis roughly perpendicular to
        # its body. This offset rotates the body yaw into the travel ("line
        # of sight") yaw before the heading error is computed:
        #   travel_yaw = body_yaw + body_to_travel_offset_rad
        # Default π/2 = travel is 90° left of the head direction. Flip the
        # sign or fine-tune empirically for your specific gait parameters.
        self.declare_parameter('body_to_travel_offset_rad', math.pi / 2)

        self.K_p = float(self.get_parameter('K_p').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.delta_max = float(self.get_parameter('delta_max').value)
        self.body_to_travel_offset_rad = float(
            self.get_parameter('body_to_travel_offset_rad').value)

        # Path-following carrot (desired point) from trajectory_publisher.
        self._latest_target: tuple[float, float] | None = None
        self._latest_com: tuple[float, float] | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Marker, '/snake/desired_position', self._on_desired_position, 10)
        # Mass-weighted body CoM, computed by center_of_mass_calculator.
        self.create_subscription(
            Marker, '/snake/center_of_mass', self._on_com, 10)

        self.delta_pub = self.create_publisher(
            Float64, '/movement_controller/delta', 10)

        # RViz visualization of the computed travel ("line of sight") yaw.
        self.travel_yaw_marker_pub = self.create_publisher(
            Marker, '/snake/travel_yaw_marker', 10)

        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.get_logger().info(
            f'Trajectory tracker started: K_p={self.K_p}, '
            f'delta_max={math.degrees(self.delta_max):.1f}°, '
            f'body→travel offset='
            f'{math.degrees(self.body_to_travel_offset_rad):.1f}°'
        )

    def _on_desired_position(self, msg: Marker) -> None:
        self._latest_target = (msg.pose.position.x, msg.pose.position.y)

    def _on_com(self, msg: Marker) -> None:
        self._latest_com = (msg.pose.position.x, msg.pose.position.y)

    def _body_yaw_world(self) -> float | None:
        """Look up world-frame xy for each motor link and fit PCA yaw."""
        positions: list[tuple[float, float]] = []
        head_xy: tuple[float, float] | None = None
        for frame in BODY_LINKS:
            try:
                tr = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            xy = (tr.transform.translation.x, tr.transform.translation.y)
            positions.append(xy)
            if frame == HEAD_LINK:
                head_xy = xy
        if head_xy is None:
            return None
        return pca_yaw_2d(positions, head_xy)

    def _tick(self) -> None:
        if self._latest_target is None or self._latest_com is None:
            return
        body_yaw = self._body_yaw_world()
        if body_yaw is None:
            return
        # Sidewinding travels perpendicular (or near-perpendicular) to the
        # body axis, so compare the heading-to-target against the *travel*
        # axis, not the raw body axis. Otherwise zero body-frame error
        # corresponds to drifting sideways past the target.
        travel_yaw = wrap_to_pi(body_yaw + self.body_to_travel_offset_rad)

        target_x, target_y = self._latest_target
        com_x, com_y = self._latest_com

        # Relative yaw error: angle from CoM to target minus travel heading.
        # Positive = target is to the left of the travel direction.
        heading_to_target = math.atan2(target_y - com_y, target_x - com_x)
        heading_error = wrap_to_pi(heading_to_target - travel_yaw)

        # Sign convention note: with alpha_distribution =
        # [2.5, 1.5, 0.5, -0.5, -1.5, -2.5] (anterior-positive), a positive δ
        # amplifies anterior joints — per the paper this rotates the snake
        # toward the *posterior* (reduced-amplitude) side. If the snake turns
        # the wrong way during tuning, flip the sign of K_p.
        delta_raw = self.K_p * heading_error
        delta = max(-self.delta_max, min(self.delta_max, delta_raw))

        self.delta_pub.publish(Float64(data=delta))
        self._publish_travel_yaw_marker(com_x, com_y, travel_yaw)

    def _publish_travel_yaw_marker(self, x: float, y: float, yaw: float) -> None:
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.ns = 'travel_yaw'
        msg.id = 0
        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.25
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        # ARROW with pose+orientation: scale.x = length, y/z = shaft/head width.
        msg.scale.x = 0.30
        msg.scale.y = 0.03
        msg.scale.z = 0.03
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 0.9
        self.travel_yaw_marker_pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
