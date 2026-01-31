# Snake-like wheel-less mobile robot simulation

Snake-like robots are a type of robot inspired by nature. Snake movement is interesting for robotics purposes because these animals can traverse various terrains, avoid obstacles, and crawl into tight spaces. That provides a lot of potential usage possibilities; for example, searching post-earthquake areas for people to rescue.

## Technical Stack

- **ROS2 Version**: Jazzy Jalisco
- **Gazebo**: Gazebo Harmonic (gz-harmonic) - compatible with ROS2 Jazzy
- **CAD to URDF**: [Onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) - automated URDF export from Onshape CAD models
- **Python**: Python 3

## Features

- Full simulation environment for snake-like robot locomotion
- URDF model generated from Onshape CAD design
- Gazebo world configurations for testing different terrains
- ROS2 controllers for joint actuation
- Development platform for novel control algorithms

## Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- ROS2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3.12+

## Installation
```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/fszkudlarek/snake_robot_simulation.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select snake_robot_simulation

# Source the workspace
source install/setup.bash
```

## Usage
```bash
# Launch the simulation
ros2 launch snake_robot_simulation <launch_file_name>.launch.py
```

## Project Structure

- `config/` - Configuration files for controllers and parameters
- `launch/` - ROS2 launch files
- `onshape_to_robot/` - Onshape-to-robot conversion files
- `urdf/` - Robot URDF description files
- `worlds/` - Gazebo world files
- `snake_sim/` - Python package for simulation nodes

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Onshape-to-robot tool for CAD-to-URDF conversion