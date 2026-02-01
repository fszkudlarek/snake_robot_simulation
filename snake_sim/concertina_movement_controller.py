import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class ConcertinaMovementController(Node):
    def __init__(self):
        super().__init__('concertina_movement_controller_node')

        # Publisher to ros2_control joint group controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/movement_controller/commands',
            10
        )

        # ---- PARAMETERS ----
        self.declare_parameter('joint_count', 6)
        self.declare_parameter('publish_rate', 50.0)   # Hz

        self.joint_count = self.get_parameter(
            'joint_count').value
        self.publish_rate = self.get_parameter(
            'publish_rate').value

        self.start_time = time.time()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Concertina movement controller started with {self.joint_count} joints")
        
        self.phase = 0
        self.phase_duration_s = 3

    def update(self):
        t = time.time() - self.start_time
        phase = int((t / self.phase_duration) % 4)

        msg = Float64MultiArray()
        msg.data = []

        for i in range(self.joint_count):
            if self.phase == 0:
                # 0
                if i < 1:
                    angle = 0
                # 1
                elif i == 1:
                    angle = -math.pi/4
                # 2, 3, ....
                else:
                    if i % 2 == 0:
                        angle = math.pi/2
                    else:
                        angle = -math.pi/2
                msg.data.append(angle)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ConcertinaMovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
