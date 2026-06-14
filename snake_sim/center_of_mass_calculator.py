import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np


ROOT_LINK = 'motor_with_onshape_mounting'


def quaternion_rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    qv = np.array([q[0], q[1], q[2]])
    t = 2.0 * np.cross(qv, v)
    return v + q[3] * t + np.cross(qv, t)


class CenterOfMassCalculator(Node):
    def __init__(self):
        super().__init__('center_of_mass_calculator')

        # Marker/compute rate. Stays comfortably above the 20 Hz that
        # robot_body_logger samples the marker at, but far below the old
        # 200 Hz that used to saturate this node's executor.
        self.declare_parameter('publish_rate', 50.0)
        # The whole accumulated path is republished each time, so this is kept
        # deliberately low — it only feeds the RViz trail and the one-shot
        # scene_snapshot grab, neither of which needs a high rate.
        self.declare_parameter('path_publish_rate', 5.0)
        publish_rate = self.get_parameter('publish_rate').value
        path_publish_rate = self.get_parameter('path_publish_rate').value

        # TF2 for inter-link transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Model world pose from Gazebo odometry, kept together with its stamp so
        # the body shape can be looked up at the same instant (see update()).
        self.model_pose = None
        self.model_stamp = None
        self.create_subscription(
            Odometry, '/model/snake/odometry', self._odom_callback, 10
        )

        # Publisher for CoM marker
        self.com_pub = self.create_publisher(
            Marker, '/snake/center_of_mass', 10
        )

        # Publisher for actual CoM path trace
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(
            Path, '/snake/actual_trajectory', latched_qos
        )
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        # Latest computed world-frame CoM (x, y), consumed by the path timer.
        self._latest_com_world = None

        # Link data from SDF: (name, mass, com_offset_xyz)
        self.link_data = self._build_link_data()
        self.total_mass = sum(mass for _, mass, _ in self.link_data)

        # Compute + publish the marker fast; republish the growing path slowly.
        self.timer = self.create_timer(1.0 / publish_rate, self.update)
        self.path_timer = self.create_timer(
            1.0 / path_publish_rate, self._publish_path
        )

        self.get_logger().info(
            f'CoM calculator started: {len(self.link_data)} links, '
            f'total mass: {self.total_mass:.4f} kg'
        )

    def _odom_callback(self, msg):
        self.model_pose = msg.pose.pose
        self.model_stamp = msg.header.stamp

    def _build_link_data(self):
        """Link masses and CoM offsets extracted from snake.sdf."""
        links = []

        # Motor segment 1 (root link)
        links.append(('motor_with_onshape_mounting', 0.146495,
                       np.array([6.26697e-10, 0.0126367, 0.0331957])))

        # Motor segments 2-6
        for i in range(2, 7):
            links.append((f'motor_with_onshape_mounting_{i}', 0.146495,
                           np.array([-6.26697e-10, -0.0272931, -0.0268243])))

        # Motor segment 7 (tail)
        links.append(('motor_with_no_arms', 0.120435,
                       np.array([-7.62304e-10, -0.0147254, -0.0275415])))

        # Inner adhesive pads (7 total, ~0.003 kg each)
        inner_com = np.array([0.0078, 0.0157555, 0.0])
        links.append(('inner_adhesive_pad', 0.00312037, inner_com.copy()))
        for i in range(2, 8):
            links.append((f'inner_adhesive_pad_{i}', 0.00312037, inner_com.copy()))

        # Outer adhesive pads (7 total, ~0.004 kg each)
        outer_com = np.array([0.0117, 0.0235675, 0.0])
        links.append(('outer_adhesive_pad', 0.00381674, outer_com.copy()))
        for i in range(2, 8):
            links.append((f'outer_adhesive_pad_{i}', 0.00381674, outer_com.copy()))

        return links

    def _compute_com_root(self, lookup_time):
        """Mass-weighted CoM in the root link frame at lookup_time.

        Returns an (x, y, z) array, or None if the full body TF is not
        available at that time. The lookup is non-blocking (timeout 0); the
        caller decides how to fall back.
        """
        com_root = np.zeros(3)
        mass_accum = 0.0

        for link_name, mass, com_offset in self.link_data:
            try:
                transform = self.tf_buffer.lookup_transform(
                    ROOT_LINK, link_name, lookup_time
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None

            t = transform.transform.translation
            pos = np.array([t.x, t.y, t.z])

            r = transform.transform.rotation
            q = (r.x, r.y, r.z, r.w)
            rotated_offset = quaternion_rotate(q, com_offset)

            link_com = pos + rotated_offset
            com_root += mass * link_com
            mass_accum += mass

        if mass_accum == 0.0:
            return None

        return com_root / mass_accum

    def update(self):
        if self.model_pose is None:
            return

        # Snapshot the latest odometry pose and its stamp together, so we map a
        # body shape from the *same* instant into the world frame. Combining a
        # stale world pose with a fresh body shape (or vice versa) is what made
        # the CoM swing back and forth during the sidewinding gait.
        model_pose = self.model_pose
        model_stamp = self.model_stamp

        # Step 1: CoM in root link frame, looked up at the odometry's stamp.
        # Fall back to the latest buffered TF when that exact time isn't
        # available yet (e.g. odom slightly ahead of the body TF).
        com_root = self._compute_com_root(rclpy.time.Time.from_msg(model_stamp))
        if com_root is None:
            com_root = self._compute_com_root(rclpy.time.Time())
        if com_root is None:
            return

        # Step 2: transform to world frame using Gazebo odometry
        p = model_pose.position
        o = model_pose.orientation
        world_pos = np.array([p.x, p.y, p.z])
        world_q = (o.x, o.y, o.z, o.w)
        com_world = quaternion_rotate(world_q, com_root) + world_pos

        self._latest_com_world = (com_world[0], com_world[1])

        # Publish CoM cylinder marker
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.ns = 'center_of_mass'
        msg.id = 0
        msg.type = Marker.CYLINDER
        msg.action = Marker.ADD
        msg.pose.position.x = com_world[0]
        msg.pose.position.y = com_world[1]
        msg.pose.position.z = 0.25
        msg.scale.x = 0.01
        msg.scale.y = 0.01
        msg.scale.z = 0.05
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 0.8
        self.com_pub.publish(msg)

    def _publish_path(self):
        """Append the latest CoM to the trail and republish it (low rate)."""
        if self._latest_com_world is None:
            return

        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self._latest_com_world[0]
        pose.pose.position.y = self._latest_com_world[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.actual_path.header.stamp = now
        self.actual_path.poses.append(pose)
        self.path_pub.publish(self.actual_path)


def main():
    rclpy.init()
    node = CenterOfMassCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
