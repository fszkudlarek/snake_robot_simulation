"""
Headless ROS 2 node that logs robot body link positions and the center of mass
to a CSV, one row per tick. Intended to run for the duration of a simulation
to capture the full trajectory (useful for rotational / orientation analysis).

Output CSV columns:
    time,
    <frame_1>_x, <frame_1>_y, ..., <frame_N>_x, <frame_N>_y,
    com_x, com_y

Missing samples (TF lookup failure, no COM received yet) are written as empty cells.

Usage:
  ros2 run snake_sim robot_body_logger --ros-args \
      -p output_path:=/path/to/body_trajectory.csv \
      -p sample_rate_hz:=20.0
"""
import csv

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import tf2_ros


# Same link set as scene_snapshot.py — keep in sync if the robot changes.
LINK_FRAMES = (
    ['motor_with_onshape_mounting']
    + [f'motor_with_onshape_mounting_{i}' for i in range(2, 7)]
    + ['motor_with_no_arms']
)


class RobotBodyLogger(Node):
    def __init__(self):
        super().__init__('robot_body_logger')

        self.declare_parameter('output_path', 'robot_body_trajectory.csv')
        self.declare_parameter('sample_rate_hz', 20.0)

        self.output_path = self.get_parameter('output_path').value
        sample_rate = float(self.get_parameter('sample_rate_hz').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_com = None
        self.create_subscription(
            Marker, '/snake/center_of_mass', self._com_cb, 10)

        self._file = open(self.output_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        header = ['time']
        for frame in LINK_FRAMES:
            header.extend([f'{frame}_x', f'{frame}_y'])
        header.extend(['com_x', 'com_y'])
        self._writer.writerow(header)
        self._file.flush()

        self.start_time = None
        self.create_timer(1.0 / sample_rate, self._tick)

        self.get_logger().info(
            f'Robot body logger: sampling at {sample_rate} Hz, '
            f'writing to {self.output_path}')

    def _com_cb(self, msg):
        self.latest_com = (msg.pose.position.x, msg.pose.position.y)

    def _tick(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        row = [f'{t:.6f}']
        for frame in LINK_FRAMES:
            try:
                tr = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time())
                p = tr.transform.translation
                row.extend([p.x, p.y])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                row.extend(['', ''])

        if self.latest_com is not None:
            row.extend([self.latest_com[0], self.latest_com[1]])
        else:
            row.extend(['', ''])

        self._writer.writerow(row)
        # Flush every tick so SIGTERM/SIGKILL loses at most one row.
        self._file.flush()

    def close(self):
        if self._file and not self._file.closed:
            self._file.close()


def main():
    rclpy.init()
    node = RobotBodyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
