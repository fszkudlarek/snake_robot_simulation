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
            glob('launch/*')),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'assets'),
            glob('urdf/assets/*')),

        # ADD: SDF files
        (os.path.join('share', package_name, 'sdf', 'snake'),
            glob('sdf/*.sdf') + ['sdf/model.config']),
        (os.path.join('share', package_name, 'sdf', 'snake', 'assets'),
            glob('sdf/assets/*')),

        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),

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
            'movement_controller = snake_sim.movement_controller:main',
            'concertina_movement_controller = snake_sim.concertina_movement_controller:main',
        ],
    },
)
