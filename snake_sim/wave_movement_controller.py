import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class WaveMovementController(Node):
    def __init__(self):
        super().__init__('wave_movement_controller_node')

        # Publisher to ros2_control joint group controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/movement_controller/commands',
            10
        )

        # ---- PARAMETERS ----
        self.declare_parameter('swivel_joint_count', 6)
        self.declare_parameter('sliding_pad_joint_count', 7)
        self.declare_parameter('publish_rate', 50.0)   # Hz

        self.swivel_joint_count = self.get_parameter(
            'swivel_joint_count').value
        self.sliding_pad_joint_count = self.get_parameter(
            'sliding_pad_joint_count').value
        self.publish_rate = self.get_parameter(
            'publish_rate').value

        self.start_time = time.time()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update
        )

        self.get_logger().info(
            f"Concertina movement controller started with:\n\
                {self.swivel_joint_count} swivel joints\n\
                {self.sliding_pad_joint_count} x2 sliding joints")
        
        self.phase = 0
        self.phase_duration_s = 3
        self.number_of_phases = 5 + self.sliding_pad_joint_count

        # send initial position command and sleep 1 second in order to get there
        self.go_to_initial_position()

    def update(self):
        t = time.time() - self.start_time
        self.phase = int((t / self.phase_duration_s) % self.number_of_phases)

        msg = Float64MultiArray()
        msg.data = []
        # swivel joints (between modules)
        for i in range(self.swivel_joint_count):
            magic_ration = self.phase - 2*i
            if 1 <= magic_ration < 3:
                angle = math.pi/4
            elif 3 <= magic_ration < 5:
                angle = -math.pi/4
            else:
                angle = 0
            msg.data.append(angle)
        # sliding joints -- outer HIGH friction pads
        # the value is in coordinates with axis pointing to the top of the robot
        # that means, values > 0 will hide the pad,
        # and values < 0 will stick out the pad
        for i in range(self.sliding_pad_joint_count):
            magic_ration = self.phase - 2*i
            if 0 <= magic_ration < 4:
                protrusion_outer = 1
            else:
                protrusion_outer = -1
            msg.data.append(protrusion_outer)
        # sliding joints -- inner LOW friction pads
        for i in range(self.sliding_pad_joint_count):
            magic_ration = self.phase - 2*i
            if 0 <= magic_ration < 4:
                protrusion_inner = -1
            else:
                protrusion_inner = 1
            msg.data.append(protrusion_inner)

        self.publisher.publish(msg)

    # Initial position: all sviwel joints at 0 angle (snake is straight)
    # hight friction pads out, low friction pads in
    def go_to_initial_position(self):
        msg = Float64MultiArray()
        msg.data = []
        # swivel joints (between modules)
        for i in range(self.swivel_joint_count):
            angle = 0
            msg.data.append(angle)
        # sliding joints -- outer HIGH friction pads
        for i in range(self.sliding_pad_joint_count):
            protrusion_outer = -1
            msg.data.append(protrusion_outer)
        # sliding joints -- inner LOW friction pads
        for i in range(self.sliding_pad_joint_count):
            protrusion_inner = 1
            msg.data.append(protrusion_inner)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = WaveMovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
