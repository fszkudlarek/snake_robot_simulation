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

        # assumes, that number of sliding pads is equal to
        # number of swivel joints plus one 
        for i in range(self.sliding_pad_joint_count):
            prev_angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - (i-1) * self.phase_offset
            )            
            next_angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - i * self.phase_offset
            )
            average_angle = (prev_angle + next_angle) / 2
            # outer HIGH friction pads: retracting (1)
            if average_angle >= 0:
                msg.data.append(1.0)
            # outer HIGH friction pads: protruding (-1)
            else:
                msg.data.append(-1.0)

        for i in range(self.sliding_pad_joint_count):
            prev_angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - (i-1) * self.phase_offset
            )            
            next_angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - i * self.phase_offset
            )
            average_angle = (prev_angle + next_angle) / 2
            # inner LOW friction pads: protruding (-1)
            if average_angle >= 0:
                msg.data.append(-1.0)
            # inner LOW friction pads: retracting (1)
            else:
                msg.data.append(1.0)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
