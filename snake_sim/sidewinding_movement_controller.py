import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import math

SVIWEL_JOINT_TO_PREV_FRICTION_PAD_DISTANCE = 115.25
DISTANCE_BETWEEN_JOINTS = 120
PAD_SHIFT_RELATIVE_MAGIC_NUMBER = SVIWEL_JOINT_TO_PREV_FRICTION_PAD_DISTANCE/DISTANCE_BETWEEN_JOINTS

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller_node')

        # Publisher to ros2_control joint group controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/movement_controller/commands',
            10
        )

        # ---- PARAMETERS (H2 solution space) ----
        self.declare_parameter('swivel_joint_count', 6)
        self.declare_parameter('sliding_pad_joint_count', 7)
        self.declare_parameter('A_v', 1.0)                       # vertical wave amplitude (radians) -- only sign matters
        self.declare_parameter('A_h', math.pi / 6)                       # horizontal wave amplitude (radians)
        self.declare_parameter('delta_phi_v', 1.2)                # vertical inter-module phase diff (radians)
        self.declare_parameter('delta_phi_h', 1.2)                # horizontal inter-module phase diff (radians)
        self.declare_parameter('delta_phi_vh', math.pi / 2)      # vertical-to-horizontal phase offset (radians)
        self.declare_parameter('O_v', 0.0)                       # vertical wave phase offset (radians)
        self.declare_parameter('O_h', 0.0)                       # horizontal wave offset (radians)
        self.declare_parameter('T', 5.0)                         # wave period (seconds)
        self.declare_parameter('publish_rate', 50.0)             # Hz

        # ---- Asymmetric amplitude distribution (eq. 3.26, 3.27) ----
        # alpha_distribution: per-joint weighting α_i, length must equal swivel_joint_count.
        # delta: steering correction magnitude δ — updated at runtime by the trajectory
        #        tracking controller via the /movement_controller/delta topic.
        # delta_offset: feed-forward bias compensating systematic heading drift.
        # With delta=0 and delta_offset=0 the per-joint amplitude reduces to A_h,
        # i.e. the original symmetric sidewinding gait.
        self.declare_parameter('alpha_distribution', [2.5, 1.5, 0.5, -0.5, -1.5, -2.5])
        self.declare_parameter('delta', 0.0)
        self.declare_parameter('delta_offset', 0.0)
        # Saturation bounds (eq. 3.29, 3.30). Paper defaults for COBRA in radians:
        # a_min = 5° ≈ 0.0873, a_max = 50° ≈ 0.8727, delta_max = 15° ≈ 0.2618.
        self.declare_parameter('a_min', 5.0 * math.pi / 180.0)
        self.declare_parameter('a_max', 50.0 * math.pi / 180.0)
        self.declare_parameter('delta_max', 15.0 * math.pi / 180.0)

        self.swivel_joint_count = self.get_parameter(
            'swivel_joint_count').value
        self.sliding_pad_joint_count = self.get_parameter(
            'sliding_pad_joint_count').value
        self.A_v = self.get_parameter('A_v').value
        self.A_h = self.get_parameter('A_h').value
        self.delta_phi_v = self.get_parameter('delta_phi_v').value
        self.delta_phi_h = self.get_parameter('delta_phi_h').value
        self.delta_phi_vh = self.get_parameter('delta_phi_vh').value
        self.O_v = self.get_parameter('O_v').value
        self.O_h = self.get_parameter('O_h').value
        self.T = self.get_parameter('T').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.alpha_distribution = list(
            self.get_parameter('alpha_distribution').value)
        self.delta_offset = float(self.get_parameter('delta_offset').value)
        self.a_min = float(self.get_parameter('a_min').value)
        self.a_max = float(self.get_parameter('a_max').value)
        self.delta_max = float(self.get_parameter('delta_max').value)
        self.delta = self._clip_delta(
            float(self.get_parameter('delta').value))

        if len(self.alpha_distribution) != self.swivel_joint_count:
            raise ValueError(
                f"alpha_distribution length ({len(self.alpha_distribution)}) "
                f"must equal swivel_joint_count ({self.swivel_joint_count})")

        # Closed-loop steering input. The trajectory tracking controller publishes
        # an updated δ on this topic (eq. 3.26); δ_offset and α_i stay fixed.
        self.delta_subscription = self.create_subscription(
            Float64,
            '/movement_controller/delta',
            self._on_delta_update,
            10
        )

        self.start_time = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Movement controller started with {self.swivel_joint_count} swivel joints"
            f" and {self.sliding_pad_joint_count} x2 sliding pad joints")

    def _on_delta_update(self, msg: Float64) -> None:
        self.delta = self._clip_delta(float(msg.data))

    def _clip_delta(self, value: float) -> float:
        # Eq. 3.30: δ = clip(δ_raw, -δ_max, +δ_max)
        return max(-self.delta_max, min(self.delta_max, value))

    def update(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        msg = Float64MultiArray()
        msg.data = []

        # Horizontal wave — swivel/yaw joints — H2 equation (4) with per-joint
        # amplitude from eq. 3.26 saturated by eq. 3.29:
        #   A_h_i = clip(A_h + α_i · δ + δ_offset, a_min, a_max)
        # φ_h_i(t) = A_h_i * sin(2π/T * t + (i-1) * ΔΦ_h + ΔΦ_vh) + O_h
        for i in range(self.swivel_joint_count):
            A_h_i_raw = (
                self.A_h
                + self.alpha_distribution[i] * self.delta
                + self.delta_offset
            )
            A_h_i = max(self.a_min, min(self.a_max, A_h_i_raw))
            angle = A_h_i * math.sin(
                (2.0 * math.pi / self.T) * t
                + i * self.delta_phi_h
                + self.delta_phi_vh
            ) + self.O_h
            msg.data.append(angle)

        # Vertical wave — friction pads — H2 equation (3), thresholded:
        # φ_v_i(t) = A_v * sin(2π/T * t + (i-1) * ΔΦ_v)
        # Pad sits between joints → midpoint index (i - 0.5)
        outer_pad_values = []
        inner_pad_values = []
        for i in range(self.sliding_pad_joint_count):
            vertical_signal = self.A_v * math.sin(
                (2.0 * math.pi / self.T) * t
                + (i - PAD_SHIFT_RELATIVE_MAGIC_NUMBER) * self.delta_phi_v
            ) + self.O_v
            if vertical_signal >= 0:
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
