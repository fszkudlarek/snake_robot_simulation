from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'snake_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py') + glob('launch/*.xml') + glob('launch/*.yaml')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'assets'),
            glob('urdf/assets/*')),

        # SDF files (for Gazebo)
        (os.path.join('share', package_name, 'sdf', 'snake'),
            glob('sdf/*.sdf') + ['sdf/model.config']),
        (os.path.join('share', package_name, 'sdf', 'snake', 'assets'),
            glob('sdf/assets/*')),

        # SDF assets also at sdf/assets/ (for URDF package:// mesh resolution)
        (os.path.join('share', package_name, 'sdf', 'assets'),
            glob('sdf/assets/*')),

        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world') + glob('worlds/*.sdf')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Install RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filip',
    maintainer_email='fszkudlarek6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'basic_sinusoidal_movement_controller = snake_sim.basic_sinusoidal_movement_controller:main',
            'concertina_movement_controller = snake_sim.concertina_movement_controller:main',
            'wave_movement_controller = snake_sim.wave_movement_controller:main',
            'sidewinding_movement_controller = snake_sim.sidewinding_movement_controller:main',
            'center_of_mass_calculator = snake_sim.center_of_mass_calculator:main',
            'odometry_tf_broadcaster = snake_sim.odometry_tf_broadcaster:main',
            'trajectory_publisher = snake_sim.trajectory_publisher:main',
            'trajectory_tracker = snake_sim.trajectory_tracker:main',
            'scene_snapshot = snake_sim.scene_snapshot:main',
            'robot_body_logger = snake_sim.robot_body_logger:main',
        ],
    },
)
