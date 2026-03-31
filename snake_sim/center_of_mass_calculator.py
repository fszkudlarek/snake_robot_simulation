import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
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

        self.declare_parameter('publish_rate', 50.0)
        publish_rate = self.get_parameter('publish_rate').value

        # TF2 for inter-link transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Model world pose from Gazebo odometry
        self.model_pose = None
        self.create_subscription(
            Odometry, '/model/snake/odometry', self._odom_callback, 10
        )

        # Publisher
        self.com_pub = self.create_publisher(
            Marker, '/snake/center_of_mass', 10
        )

        # Link data from SDF: (name, mass, com_offset_xyz)
        self.link_data = self._build_link_data()
        self.total_mass = sum(mass for _, mass, _ in self.link_data)

        self.timer = self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            f'CoM calculator started: {len(self.link_data)} links, '
            f'total mass: {self.total_mass:.4f} kg'
        )

    def _odom_callback(self, msg):
        self.model_pose = msg.pose.pose

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

    def update(self):
        if self.model_pose is None:
            return

        # Step 1: compute CoM in root link frame using TF
        com_root = np.zeros(3)
        mass_accum = 0.0

        for link_name, mass, com_offset in self.link_data:
            try:
                transform = self.tf_buffer.lookup_transform(
                    ROOT_LINK, link_name, rclpy.time.Time()
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            t = transform.transform.translation
            pos = np.array([t.x, t.y, t.z])

            r = transform.transform.rotation
            q = (r.x, r.y, r.z, r.w)
            rotated_offset = quaternion_rotate(q, com_offset)

            link_com = pos + rotated_offset
            com_root += mass * link_com
            mass_accum += mass

        if mass_accum == 0.0:
            return

        com_root /= mass_accum

        # Step 2: transform to world frame using Gazebo odometry
        p = self.model_pose.position
        o = self.model_pose.orientation
        world_pos = np.array([p.x, p.y, p.z])
        world_q = (o.x, o.y, o.z, o.w)
        com_world = quaternion_rotate(world_q, com_root) + world_pos

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
        msg.scale.z = 0.5
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 0.8
        self.com_pub.publish(msg)


def main():
    rclpy.init()
    node = CenterOfMassCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
