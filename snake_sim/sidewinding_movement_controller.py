import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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

        # ---- PARAMETERS ----
        self.declare_parameter('swivel_joint_count', 6)
        self.declare_parameter('sliding_pad_joint_count', 7)
        self.declare_parameter('amplitude', 0.8)       # radians
        self.declare_parameter('frequency', 0.2)       # Hz
        self.declare_parameter('phase_offset', 1.2)    # radians
        self.declare_parameter('friction_phase_offset', math.pi / 2)  # radians
        self.declare_parameter('publish_rate', 50.0)   # Hz

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

        self.start_time = time.time()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Movement controller started with {self.swivel_joint_count} swivel joints"
            f" and {self.sliding_pad_joint_count} x2 sliding pad joints")

    def update(self):
        t = time.time() - self.start_time

        msg = Float64MultiArray()
        msg.data = []

        for i in range(self.swivel_joint_count):
            angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - i * self.phase_offset
            )
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
