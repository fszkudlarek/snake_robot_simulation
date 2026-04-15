import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('snake_sim')
    
    # IMPORTANT: Set Gazebo resource path so it can find meshes
    sdf_dir = os.path.join(pkg_share, 'sdf')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = sdf_dir + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = sdf_dir

    # Path to SDF file
    sdf_path = os.path.join(pkg_share, 'sdf', 'snake', 'snake.sdf')

    # Path to URDF file (needed for robot_state_publisher and RViz)
    urdf_path = os.path.join(pkg_share, 'urdf', 'snake.urdf')

    # Path to RViz config
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'snake.rviz')

    # Path to world file
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    # Launch Gazebo headless (no GUI) — using RViz for visualization instead
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -s ', world_path]}.items(),
    )

    # Robot State Publisher — publishes URDF to /robot_description and TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'snake',
            '-file', sdf_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        output='screen',
    )
    
    # Bridge for clock and model odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/snake/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    
    # Load joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Load movement controller
    movement_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['movement_controller'],
        output='screen',
    )
    
    basic_sinusoidal_controller = Node(
        package='snake_sim',
        executable='basic_sinusoidal_movement_controller',
        name='movement_controller_node',
        output='screen',
    )

    # Start the concertina movement controller node
    concertina_controller = Node(
        package='snake_sim',
        executable='concertina_movement_controller',
        name='movement_controller_node',
        output='screen',
    )

    wave_controller = Node(
        package='snake_sim',
        executable='wave_movement_controller',
        name='movement_controller_node',
        output='screen',
    )

    sidewinding_controller = Node(
        package='snake_sim',
        executable='sidewinding_movement_controller',
        name='movement_controller_node',
        output='screen',
    )

    center_of_mass_calculator = Node(
        package='snake_sim',
        executable='center_of_mass_calculator',
        name='center_of_mass_calculator',
        output='screen',
    )

    # Broadcast odom -> root link TF from Gazebo odometry
    odometry_tf_broadcaster = Node(
        package='snake_sim',
        executable='odometry_tf_broadcaster',
        name='odometry_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        movement_controller_spawner,
        sidewinding_controller,
        center_of_mass_calculator,
        odometry_tf_broadcaster,
        rviz,
    ])