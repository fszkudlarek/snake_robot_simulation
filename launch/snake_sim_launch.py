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
    
    # Path to world file
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(sdf_path).read()}]
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
    
    # Bridge for clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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
    
    # Start the concertina movement controller node
    concertina_controller = Node(
        package='snake_sim',
        executable='wave_movement_controller',
        name='movement_controller_node',
        output='screen',
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        movement_controller_spawner,
        concertina_controller,
    ])