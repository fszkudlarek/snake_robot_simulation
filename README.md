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

## References

This work is inspired by the following research on snake-like robotics:

1. **Hirose, S.** (1993). *Biologically inspired robots: snake-like locomotors and manipulators*. Oxford University Press.

2. **Ma, S.** (1999). Analysis of snake movement forms for realization of snake-like robots. *Proceedings - IEEE International Conference on Robotics and Automation*, 4, 3007–3013. [DOI: 10.1109/ROBOT.1999.774054](https://doi.org/10.1109/ROBOT.1999.774054)

3. **Bayraktaroglu, Z., Kilicarslan, A., & Kuzucu, A.** (2006). Design and Control of Biologically Inspired Wheel-less Snake-like Robot. *Proceedings of the IEEE International Conference on Biomedical Robotics and Biomechatronics*, 1001–1006. [DOI: 10.1109/BIOROB.2006.1639222](https://doi.org/10.1109/BIOROB.2006.1639222)

4. **Liljebäck, P., Pettersen, K.Y., Stavdahl, Ø., & Gravdahl, J.** (2012). A review on modelling, implementation, and control of snake robots. *Robotics and Autonomous Systems*, 60, 29–40. [DOI: 10.1016/j.robot.2011.08.010](https://doi.org/10.1016/j.robot.2011.08.010)

5. **Walker, I.D., Choset, H., & Chirikjian, G.S.** (2016). Snake-Like and Continuum Robots. In B. Siciliano & O. Khatib (Eds.), *Springer Handbook of Robotics* (2nd ed., pp. 481–498). Springer International Publishing.

6. **Li, N., Wang, F., Ren, S., Cheng, X., Wang, G., & Li, P.** (2025). A Review on the Recent Development of Planar Snake Robot Control and Guidance. *Mathematics*, 13(2), 189. [DOI: 10.3390/math13020189](https://doi.org/10.3390/math13020189)

7. **Sobhani, M., & Zarif Loloei, A.** (2025). Design and Implementation of a Neural Network-Based PD Controller for Adaptive Motion Control of a Novel Snake-like Robot.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Onshape-to-robot tool for CAD-to-URDF conversion