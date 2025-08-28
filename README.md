# VSSS Robot Soccer Simulation

A complete simulation environment for the Very Small Size Soccer (VSSS) competition using ROS Noetic and Gazebo. This simulator provides a realistic 3D environment with robot physics, ball dynamics, and field visualization for developing and testing robot soccer strategies.

**Note**: This is a simulation environment only - no control strategies or autonomous behaviors are implemented. You can control robots manually via joystick or develop your own control algorithms.

## Overview

This simulation environment includes:
- **3D Soccer Field**: Complete VSSS field with accurate dimensions and markings
- **Robot Models**: Differential drive robots with realistic physics
- **Ball Physics**: Realistic ball movement and collision dynamics
- **Camera System**: Top-view camera feed for computer vision development
- **Joystick Control**: Real-time robot control using game controllers
- **Team Management**: Support for both blue and yellow teams (3 robots each)

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 20.04 LTS
- **ROS Version**: ROS Noetic
- **Hardware**: Joystick/Gamepad (optional, for manual control)

### ROS Dependencies

Install ROS Noetic Desktop Full following the [official installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

Install the required velocity controllers package:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-velocity-controllers
sudo apt-get install ros-noetic-image-transport-plugins
```

## Installation

1. **Navigate to your catkin workspace source directory:**
   ```bash
   cd ~/catkin_ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/juliodltv/simulation_vsss.git
   ```

3. **Build the workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source the workspace:**
   ```bash
   source devel/setup.bash
   ```
   
   Add this line to your `~/.bashrc` for automatic sourcing:
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   ```

## Running the Simulation

### Basic Simulation
Start the complete simulation environment:

```bash
roslaunch simulation_vsss simulation_match.launch
```

This launches:
- Gazebo world with the VSSS field
- 6 robots (3 blue, 3 yellow)
- Ball physics
- Camera system
- ROS control infrastructure

### Alternative Launch Files

- **Single robot testing:**
  ```bash
  roslaunch simulation_vsss simulation_robot.launch
  ```

- **1 vs 1 match:**
  ```bash
  roslaunch simulation_vsss simulation_vs.launch
  ```

- **Single team (3 robots):**
  ```bash
  roslaunch simulation_vsss simulation_team.launch
  ```

- **Two robots on yellow team:**
  ```bash
  roslaunch simulation_vsss simulation_offense.launch
  ```

## Joystick Control

### Supported Controllers
- USB Generic Gamepad
- Xbox 360 Controller
- Xbox One Controller (default)
- Xbox Chinese Controller

### Starting Joystick Control

The joystick will be automatically initialized when plugged in during an active simulation.

### Control Mapping

| Control | Action |
|---------|--------|
| **Left Stick** | Linear velocity (forward/backward) |
| **Right Stick** | Angular velocity (rotation) |
| **X Button** | Select Blue Team |
| **Y Button** | Select Yellow Team |
| **RB (Right Button)** | Next Robot (0→1→2→0) |
| **LB (Left Button)** | Previous Robot (2→1→0→2) |
| **START Button** | Ball Control Mode |
| **BACK Button** | Reset World |
| **D-Pad Up/Down** | Increase/Decrease Lighting |

The joystick operates in differential drive mode, using linear and angular velocities to control robot movement.

## Useful ROS Commands

### Visualization Tools

**View camera feed:**
```bash
rqt_image_view
```
Select `/camera/image_raw/compressed` topic to see the field from above.

**Visualize ROS topics and nodes:**
```bash
rqt_graph
```

**Monitor all topics:**
```bash
rostopic list
```

**Check specific topic data:**
```bash
rostopic echo /camera/image_raw
rostopic echo /blue/0/left_controller/command
```

### Robot Control via Command Line

**Control blue robot 0:**
```bash
# Left wheel
rostopic pub /blue/0/left_controller/command std_msgs/Float64 "data: 30.0"
# Right wheel  
rostopic pub /blue/0/right_controller/command std_msgs/Float64 "data: 30.0"
```

**Control yellow robot 1:**
```bash
# Left wheel
rostopic pub /yellow/1/left_controller/command std_msgs/Float64 "data: -20.0"
# Right wheel
rostopic pub /yellow/1/right_controller/command std_msgs/Float64 "data: 20.0"
```

### System Information

**Check active nodes:**
```bash
rosnode list
```

**Get node information:**
```bash
rosnode info /gazebo
```

**Service calls:**
```bash
rosservice list
rosservice call /gazebo/reset_world
```

## Project Structure

```
simulation_vsss/
├── launch/          # ROS launch files
├── urdf/            # Robot URDF models  
├── models/          # Gazebo models (field, ball, camera)
├── worlds/          # Gazebo world files
├── scripts/         # Python control scripts
├── config/          # Configuration files
└── media/           # Textures and materials
```

## Troubleshooting

### Common Issues
**No joystick response:**
- Check if joystick is detected: `ls /dev/input/js*`
   - To change the input device, modify `rosrun joy joy_node _dev:=/dev/input/jsN` where N is the joystick number.
- Verify joystick topic: `rostopic echo /joy`
- Check launch file parameters

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Based on [ThundeRatz/travesim](https://github.com/ThundeRatz/travesim)
- VSSS Competition rules and specifications
- ROS and Gazebo communities