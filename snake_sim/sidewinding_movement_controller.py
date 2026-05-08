import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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

        self.start_time = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Movement controller started with {self.swivel_joint_count} swivel joints"
            f" and {self.sliding_pad_joint_count} x2 sliding pad joints")

    def update(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        msg = Float64MultiArray()
        msg.data = []

        # Horizontal wave — swivel/yaw joints — H2 equation (4):
        # φ_h_i(t) = A_h * sin(2π/T * t + (i-1) * ΔΦ_h + ΔΦ_vh) + O_h
        for i in range(self.swivel_joint_count):
            angle = self.A_h * math.sin(
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
                + self.O_v
            )
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
