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
        self.declare_parameter('joint_count', 6)
        self.declare_parameter('amplitude', 0.4)       # radians
        self.declare_parameter('frequency', 0.8)       # Hz
        self.declare_parameter('phase_offset', 0.6)    # radians
        self.declare_parameter('publish_rate', 50.0)   # Hz

        self.joint_count = self.get_parameter(
            'joint_count').value
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
            f"Movement controller started with {self.joint_count} joints")

    def update(self):
        t = time.time() - self.start_time

        msg = Float64MultiArray()
        msg.data = []

        for i in range(self.joint_count):
            angle = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t
                - i * self.phase_offset
            )
            msg.data.append(angle)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
