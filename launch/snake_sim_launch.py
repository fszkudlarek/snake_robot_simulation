from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_name = "snake_sim"

def generate_launch_description():
    pkg_share = get_package_share_directory(package_name)

    urdf_path = os.path.join(pkg_share, "urdf", "snake.urdf")
    world_path = os.path.join(pkg_share, "worlds", "empty.world")
    controllers = os.path.join(pkg_share, "config", "controllers.yaml")

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": open(urdf_path).read(),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen"
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "snake", "-topic", "robot_description"],
        output="screen",
    )

    # Bridge for clock (optional but recommended)
    gz_ros2_bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawner nodes for controllers (delayed to allow Gazebo to initialize)
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_snake = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["movement_controller"],
        output="screen",
    )

    # Movement controller node
    movement_controller_node = Node(
        package="snake_sim",
        executable="concertina_movement_controller",
        name="movement_controller_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Delay spawners to allow Gazebo's controller_manager to initialize
    spawn_jsb_delayed = TimerAction(period=5.0, actions=[spawn_jsb])
    spawn_snake_delayed = TimerAction(period=6.0, actions=[spawn_snake])
    
    # Delay movement controller to start after controllers are spawned
    movement_controller_delayed = TimerAction(period=8.0, actions=[movement_controller_node])

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        gz_ros2_bridge_clock,
        spawn_jsb_delayed,
        spawn_snake_delayed,
        movement_controller_delayed,
    ])