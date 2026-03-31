import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry, Path
import math
import time


class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller_node')

        # Publisher to ros2_control joint group controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/movement_controller/commands',
            10
        )

        # ---- GAIT PARAMETERS ----
        self.declare_parameter('swivel_joint_count', 6)
        self.declare_parameter('sliding_pad_joint_count', 7)
        self.declare_parameter('amplitude', 0.8)       # radians
        self.declare_parameter('frequency', 0.2)       # Hz
        self.declare_parameter('phase_offset', 1.2)    # radians
        self.declare_parameter('friction_phase_offset', math.pi / 2)  # radians
        self.declare_parameter('publish_rate', 50.0)   # Hz

        # ---- TRAJECTORY TRACKING PARAMETERS ----
        self.declare_parameter('kp_cross_track', 2.0)   # gain for cross-track error
        self.declare_parameter('kh_heading', 1.0)        # gain for heading error
        self.declare_parameter('max_steering_bias', 0.4) # clamp (radians)

        self.swivel_joint_count = self.get_parameter(
            'swivel_joint_count').value
        self.sliding_pad_joint_count = self.get_parameter(
            'sliding_pad_joint_count').value
        self.amplitude = self.get_parameter(
            'amplitude').value
        self.frequency = self.get_parameter(
            'frequency').value
        self.phase_offset = self.get_parameter(
            'phase_offset').value
        self.friction_phase_offset = self.get_parameter(
            'friction_phase_offset').value
        self.publish_rate = self.get_parameter(
            'publish_rate').value
        self.kp = self.get_parameter('kp_cross_track').value
        self.kh = self.get_parameter('kh_heading').value
        self.max_bias = self.get_parameter('max_steering_bias').value

        # ---- ODOMETRY STATE ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.create_subscription(
            Odometry, '/model/snake/odometry', self._odom_callback, 10
        )

        # ---- DESIRED TRAJECTORY ----
        self.trajectory = None
        trajectory_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            Path, '/snake/desired_trajectory',
            self._trajectory_callback, trajectory_qos
        )

        self.start_time = time.time()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Movement controller started with {self.swivel_joint_count} swivel joints"
            f" and {self.sliding_pad_joint_count} x2 sliding pad joints")

    def _odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )

    def _trajectory_callback(self, msg):
        self.trajectory = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.get_logger().info(
            f'Received desired trajectory with {len(self.trajectory)} points'
        )

    def _compute_steering_bias(self):
        if self.trajectory is None or len(self.trajectory) < 2:
            return 0.0

        # Find closest point on trajectory
        px, py = self.robot_x, self.robot_y
        min_dist_sq = float('inf')
        closest_idx = 0
        for i, (tx, ty) in enumerate(self.trajectory):
            d = (px - tx) ** 2 + (py - ty) ** 2
            if d < min_dist_sq:
                min_dist_sq = d
                closest_idx = i

        # Tangent vector at closest point
        if closest_idx < len(self.trajectory) - 1:
            t_idx = closest_idx
        else:
            t_idx = closest_idx - 1
        ax, ay = self.trajectory[t_idx]
        bx, by = self.trajectory[t_idx + 1]
        tan_x = bx - ax
        tan_y = by - ay
        tan_len = math.hypot(tan_x, tan_y)
        if tan_len < 1e-9:
            return 0.0
        tan_x /= tan_len
        tan_y /= tan_len

        # Signed cross-track error
        # Cross product (tangent × displacement) — positive = robot is LEFT of path
        dx = px - ax
        dy = py - ay
        cross_track_error = tan_x * dy - tan_y * dx

        # Heading error (desired - actual), normalized to [-pi, pi]
        path_heading = math.atan2(tan_y, tan_x)
        heading_error = math.atan2(
            math.sin(path_heading - self.robot_yaw),
            math.cos(path_heading - self.robot_yaw)
        )

        # Steering bias
        # Joint axis is Z-up, positive angle = CCW = turn left.
        # Robot LEFT of path (cross_track > 0) → steer right → negative bias
        # Heading error > 0 (need to turn left) → positive bias
        bias = -self.kp * cross_track_error + self.kh * heading_error
        bias = max(-self.max_bias, min(self.max_bias, bias))
        return bias

    def update(self):
        t = time.time() - self.start_time
        steering_bias = self._compute_steering_bias()

        msg = Float64MultiArray()
        msg.data = []

        for i in range(self.swivel_joint_count):
            angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - i * self.phase_offset
            ) + steering_bias
            msg.data.append(angle)

        # Friction wave uses an independent phase offset from the bending wave
        # to control which segments grip vs slide
        outer_pad_values = []
        inner_pad_values = []
        for i in range(self.sliding_pad_joint_count):
            # Pad position is between joint i-1 and joint i;
            # use midpoint (i - 0.5) to center the friction signal
            friction_signal = math.sin(
                2.0 * math.pi * self.frequency * t
                - (i - 0.5) * self.phase_offset
                + self.friction_phase_offset
            )
            if friction_signal >= 0:
                # Grip: outer HIGH friction pads protruding, inner retracted
                outer_pad_values.append(-1.0)
                inner_pad_values.append(1.0)
            else:
                # Slide: inner LOW friction pads protruding, outer retracted
                outer_pad_values.append(1.0)
                inner_pad_values.append(-1.0)

        msg.data.extend(outer_pad_values)
        msg.data.extend(inner_pad_values)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
