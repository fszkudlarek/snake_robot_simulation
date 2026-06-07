import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz visualization'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_share = get_package_share_directory('snake_sim')

    controller_params_arg = DeclareLaunchArgument(
        'controller_params_file',
        default_value=os.path.join(pkg_share, 'config', 'default_controller_params.yaml'),
        description='Path to ROS params YAML file overriding movement controller parameters'
    )
    controller_params_file = LaunchConfiguration('controller_params_file')

    trajectory_log_arg = DeclareLaunchArgument(
        'trajectory_log_path',
        default_value='',
        description='If non-empty, stream every robot body link + COM position to '
                    'this CSV file for the full simulation duration.'
    )
    trajectory_log_path = LaunchConfiguration('trajectory_log_path')

    use_trajectory_publisher_arg = DeclareLaunchArgument(
        'use_trajectory_publisher', default_value='false',
        description='Launch trajectory_publisher (config/trajectory.yaml) so a desired '
                    'trajectory is published for visualization and offline scoring. '
                    'No trajectory_tracker is started — gait remains open-loop.'
    )
    use_trajectory_publisher = LaunchConfiguration('use_trajectory_publisher')
    
    # IMPORTANT: Set Gazebo resource path so it can find meshes
    sdf_dir = os.path.join(pkg_share, 'sdf')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = sdf_dir + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = sdf_dir

    # Path to SDF file
    sdf_path = os.path.join(pkg_share, 'sdf', 'snake', 'snake.sdf')

    # URDF filename in the package urdf/ directory. Use snake_colored.urdf
    # (from scripts/colorize_urdf.py) for per-segment colored RViz screenshots.
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file', default_value='snake.urdf',
        description='URDF filename in the package urdf/ directory '
                    '(e.g. snake_colored.urdf for colored body segments).'
    )
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('urdf_file')])

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
        parameters=[{'robot_description': ParameterValue(
            Command(['cat ', urdf_path]), value_type=str)}]
    )

    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
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
        parameters=[{'use_sim_time': True}, controller_params_file],
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

    # Optional: stream per-tick body link + COM positions to CSV for the full run.
    # Only launched when `trajectory_log_path` is non-empty.
    robot_body_logger = Node(
        package='snake_sim',
        executable='robot_body_logger',
        name='robot_body_logger',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_path': trajectory_log_path},
        ],
        condition=IfCondition(
            PythonExpression(["'", trajectory_log_path, "' != ''"])
        ),
    )

    # Optional: publish a desired trajectory (open-loop — no tracker is started).
    # Off by default; the optimizer driver turns it on so simulations have a
    # reference path for offline scoring (and RViz visualization).
    trajectory_config = os.path.join(pkg_share, 'config', 'trajectory.yaml')
    trajectory_publisher = Node(
        package='snake_sim',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, trajectory_config],
        condition=IfCondition(use_trajectory_publisher),
    )

    # Startup chain: spawn robot → load joint_state_broadcaster
    # → load movement_controller → start sidewinding controller and friends
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_mc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[movement_controller_spawner],
        )
    )
    start_app_nodes_after_mc = RegisterEventHandler(
        OnProcessExit(
            target_action=movement_controller_spawner,
            on_exit=[
                sidewinding_controller,
                center_of_mass_calculator,
                odometry_tf_broadcaster,
                robot_body_logger,
                trajectory_publisher,
            ],
        )
    )

    return LaunchDescription([
        use_rviz_arg,
        urdf_file_arg,
        controller_params_arg,
        trajectory_log_arg,
        use_trajectory_publisher_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        load_jsb_after_spawn,
        load_mc_after_jsb,
        start_app_nodes_after_mc,
        rviz,
    ])