"""
Headless ROS 2 node that captures a top-down snapshot of the snake scene
by subscribing to the same topics as RViz, then rendering with matplotlib.

Usage:
  ros2 run snake_sim scene_snapshot --ros-args -p output_path:=/path/to/screenshot.png -p wait_seconds:=5.0
"""
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless backend — no display needed
import matplotlib.pyplot as plt


# All robot link frames (for plotting the robot body)
LINK_FRAMES = (
    ['motor_with_onshape_mounting']
    + [f'motor_with_onshape_mounting_{i}' for i in range(2, 7)]
    + ['motor_with_no_arms']
)


class SceneSnapshot(Node):
    def __init__(self):
        super().__init__('scene_snapshot')

        self.declare_parameter('output_path', 'snapshot.png')
        self.declare_parameter('wait_seconds', 5.0)

        self.output_path = self.get_parameter('output_path').value
        self.wait_seconds = self.get_parameter('wait_seconds').value

        # TF for robot link positions
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to trajectory paths (transient local QoS to get latched data)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.actual_path = None
        self.desired_path = None
        self.com_marker = None

        self.create_subscription(
            Path, '/snake/actual_trajectory', self._actual_cb, latched_qos)
        self.create_subscription(
            Path, '/snake/desired_path', self._desired_cb, latched_qos)
        self.create_subscription(
            Marker, '/snake/center_of_mass', self._com_cb, 10)

        # One-shot timer: wait for data, then render and exit
        self.create_timer(self.wait_seconds, self._render_and_exit)

        self.get_logger().info(
            f'Scene snapshot: collecting data for {self.wait_seconds}s, '
            f'will save to {self.output_path}')

    def _actual_cb(self, msg):
        self.actual_path = msg

    def _desired_cb(self, msg):
        self.desired_path = msg

    def _com_cb(self, msg):
        self.com_marker = msg

    def _get_robot_link_positions(self):
        """Get world-frame positions of all motor links via TF.

        Returns a list of (frame, x, y) tuples, skipping any frames whose TF
        lookup failed so the caller can keep frame↔position alignment.
        """
        positions = []
        for frame in LINK_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform('odom', frame, rclpy.time.Time())
                p = t.transform.translation
                positions.append((frame, p.x, p.y))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        return positions

    def _csv_path(self, suffix):
        base, _ = os.path.splitext(self.output_path)
        return f'{base}_{suffix}.csv'

    def _save_xy_csv(self, path, xs, ys):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y'])
            for x, y in zip(xs, ys):
                w.writerow([x, y])

    def _save_csvs(self, robot_links):
        out_dir = os.path.dirname(self.output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        if self.desired_path and len(self.desired_path.poses) > 0:
            xs = [p.pose.position.x for p in self.desired_path.poses]
            ys = [p.pose.position.y for p in self.desired_path.poses]
            self._save_xy_csv(self._csv_path('desired_trajectory'), xs, ys)

        if self.actual_path and len(self.actual_path.poses) > 0:
            xs = [p.pose.position.x for p in self.actual_path.poses]
            ys = [p.pose.position.y for p in self.actual_path.poses]
            self._save_xy_csv(self._csv_path('actual_trajectory'), xs, ys)

        if self.com_marker:
            p = self.com_marker.pose.position
            self._save_xy_csv(self._csv_path('center_of_mass'), [p.x], [p.y])

        if robot_links:
            with open(self._csv_path('robot_body'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['frame', 'x', 'y'])
                for frame, x, y in robot_links:
                    w.writerow([frame, x, y])

    def _render_and_exit(self):
        robot_links = self._get_robot_link_positions()

        self._save_csvs(robot_links)

        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        ax.set_aspect('equal')
        ax.set_facecolor('#303030')
        fig.patch.set_facecolor('#303030')
        ax.grid(True, alpha=0.3, color='#a0a0a4', linewidth=0.5)
        ax.tick_params(colors='white')
        for spine in ax.spines.values():
            spine.set_color('#a0a0a4')

        has_data = False

        # Plot desired trajectory (green)
        if self.desired_path and len(self.desired_path.poses) > 0:
            xs = [p.pose.position.x for p in self.desired_path.poses]
            ys = [p.pose.position.y for p in self.desired_path.poses]
            ax.plot(xs, ys, color='#00ff00', linewidth=1.5, label='Desired trajectory')
            has_data = True

        # Plot actual trajectory (orange)
        if self.actual_path and len(self.actual_path.poses) > 0:
            xs = [p.pose.position.x for p in self.actual_path.poses]
            ys = [p.pose.position.y for p in self.actual_path.poses]
            ax.plot(xs, ys, color='#ff6400', linewidth=1.5, label='Actual trajectory')
            has_data = True

        # Plot center of mass marker (red dot)
        if self.com_marker:
            ax.plot(self.com_marker.pose.position.x,
                    self.com_marker.pose.position.y,
                    'o', color='red', markersize=8, label='Center of mass')
            has_data = True

        # Plot robot body (link positions connected by a line)
        if robot_links:
            rxs = [x for _, x, _ in robot_links]
            rys = [y for _, _, y in robot_links]
            ax.plot(rxs, rys, 'o-', color='#4a9eff', linewidth=3,
                    markersize=6, label='Robot body')
            has_data = True

        if not has_data:
            self.get_logger().warn('No data received — snapshot will be empty.')

        ax.legend(loc='upper left', facecolor='#404040', edgecolor='#a0a0a4',
                  labelcolor='white')
        ax.set_xlabel('X (m)', color='white')
        ax.set_ylabel('Y (m)', color='white')
        ax.set_title('Snake Robot — Top-Down View', color='white', fontsize=14)

        fig.savefig(self.output_path, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        plt.close(fig)

        self.get_logger().info(f'Snapshot saved to {self.output_path}')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = SceneSnapshot()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
