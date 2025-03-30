# ArthroBot - (5+1) DOF Desktop Robotic Arm

## Overview
**ArthroBot** is a 6-DOF desktop robotic arm designed for real-time control, motion planning, and simulation. It integrates embedded microcontroller firmware with a comprehensive ROS 2 workspace. The system facilitates switching between simulation and hardware modes, using ROS 2 for inter-component communication and MoveIt 2 for motion planning. The architecture is modular, with a clear separation between hardware control and software components. The project leverages a combination of Arduino-based firmware and ROS 2 nodes to manage the robotic arm's operation.

## Mission
ArthroBot sets out to make the world of robotics accessible with its compact, low-cost, and simple design.
1. Fully 3D printed.
2. Low-price motors (MG996R/MG995 and MG90S/SG90 servos).
3. Low-price microcontroller (Any MCU that supports USB-UART bridge to communicate with a computer).
4. ROS 2 and MoveIt 2 based (extensive community support and documentation).
5. Open-Source and free for use.

ArthroBot's mission is to provide an entry point to learn ROS 2 and MoveIt 2 on a real robot. It serves as a hands-on platform for experimenting with and developing kinematics, motion control algorithms, and logic. The robot model can be easily replaced with another with minimal effort

![image](https://github.com/user-attachments/assets/6aa054a5-7137-414c-8ad8-8b1e9e9f9500)

## Project Structure
The project is organized into two primary directories:

### Control (Embedded Firmware)
* Contains C++ code for microcontroller firmware, including servo drivers and communication protocols.
* Python control scripts for teleoperation and test purposes.
* Built using PlatformIO for deployment to the Arduino UNO R3.

### arthrobot_ws (ROS 2 Workspace)
* Contains ROS 2 packages that manage various aspects of the robotic arm's operation, including control, simulation, motion planning, and task management.

## Key Directories and Files
### `control` Directory:
* `main.cpp`: C++ code for controlling hardware.
* `platformio.ini`: PlatformIO configuration for building and deploying to Arduino.
* `include/` and `lib/`: Contain libraries for servo control, PWM, and I2C communication.

### `arthrobot_ws` Directory:
* `arthrobot_client`: Teleoperation interface using joystick input.
* `arthrobot_controller`: Hardware interface for controlling motors.
* `arthrobot_description`: URDF and RViz configuration for simulating and visualizing the robot.
* `arthrobot_interfaces`: ROS messages and actions for communication between components.
* `arthrobot_moveit`: MoveIt 2 configuration files for motion planning.
* `arthrobot_task_server`: Task management and execution server.

## System Architecture
1. **Embedded/Hardware Layer**:
  * Arduino UNO R3: Controls the servo motors and other hardware components.
  * Control Firmware: Written in C++ for embedded development, handling servo control via PWM and I2C communication.
  * Communication: The firmware communicates with the ROS 2 system via serial links.

2. **ROS 2 Core Workspace Components**:
  * `arthrobot_client`: Provides the user interface for teleoperation, sending commands like joystick inputs to control the robotic arm.
  * `arthrobot_controller`: Serves as the bridge between the Arduino-based firmware and the ROS 2 system, managing hardware control via C++.
  * `arthrobot_description`: Contains the digital twin of the robotic arm, enabling simulation and visualization via URDF and RViz.
  * `arthrobot_interfaces`: Defines ROS messages and actions, such as ArthrobotPositionCommand and ArthrobotTask, to standardize communication.
  * `arthrobot_moveit`: Configures MoveIt 2 for motion planning, enabling path planning and collision-free movement for the robotic arm.
  * `arthrobot_task_server`: Manages task execution, orchestrating higher-level commands for the robotic arm.

3. **Inter-Component Communication**:
Communication between ROS components is handled using ROS 2 topics, actions, and services.
The arthrobot_controller node interacts with the Arduino firmware for real-time motion control using `pyserial`.

## Architectural Design Principles
1. **Modular Design**: The firmware and ROS 2 packages are isolated into distinct functional units, promoting ease of maintenance and flexibility.

2. **Separation of Concerns**: Hardware control is decoupled from motion planning, simulation, and user interaction, providing clean interfaces between the system layers.

3. **Code Reuse & Configurability**: CMake, package.xml, and launch files provide flexibility, enabling easy configuration for both simulation and hardware modes.

4. **Digital Twin Integration**: The arthrobot_description package creates a digital counterpart to the physical robot, supporting simulation and visualization through RViz.

## Component Mapping
Component	Mapping in Repository
| | |
|--------------------------------------|---------------------------------------------------------------------------|
|Embedded Firmware (Arduino & Drivers) | Control/src/main.cpp, Control/platformio.ini, Control/include, Control/lib|
|Arthrobot Client (Teleoperation) |	arthrobot_ws/src/arthrobot_client (arthrobot_client/arthrobot_control_joystick.py, arthrobot_servo_client.py)|
|Arthrobot Controller (Hardware Interface) | arthrobot_ws/src/arthrobot_controller (arthrobot_controller/arthrobot_hardware_interface.hpp, arthrobot_hardware_interface.cpp)|
|Arthrobot Description (Digital Twin) |	arthrobot_ws/src/arthrobot_description (urdf files, meshes, rviz/display.rviz)|
|Arthrobot Interfaces (ROS Messages) |	arthrobot_ws/src/arthrobot_interfaces (action/ArthrobotTask.action, msg/ArthrobotPositionCommand.msg)|
|Arthrobot MoveIt (Motion Planning) |	arthrobot_ws/src/arthrobot_moveit (config files like arthrobot.srdf, kinematics.yaml, launch/moveit.launch.py)|
|Arthrobot Task Server (Task Management) |	arthrobot_ws/src/arthrobot_task_server (arthrobot_task_server.cpp)|

## Installation Guide

1. Install ROS 2 Humble
Follow the official guide to install ROS 2 Humble on Ubuntu [ROS 2 Official Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

2. Install ROS 2 Developer Tools
```bash
sudo apt install ros-dev-tools
```

3. Install MoveIt 2 for ROS 2 Humble

```bash
sudo apt install ros-humble-moveit
```

4. Set Up Automatic ROS 2 Sourcing in New Terminals

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

5. Install Cyclone DDS Middleware (Alternative to Fast DDS)
```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

6. Add Cyclone DDS to .bashrc
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

7. Install Dependencies for ArthroBot
```bash
# MoveIt Servo (for real-time motion control)
 sudo apt install ros-humble-moveit-servo
# Position Controllers (for controlling the robot's joints)
sudo apt install ros-humble-position-controllers
# Controller Manager (for managing robot controllers)
sudo apt install ros-humble-controller-manager
```
8. Install Python Modules
```bash
# Install pip (if not already installed)
sudo apt install python3-pip
# Install the pyserial module
pip install pyserial
# Install the PS4 Controller module
sudo pip install pyPS4Controller
```

9. Clone the ArthroBot GitHub Repository
```bash
git clone https://github.com/Fadi-Eid/ArthroBot.git
```

10. Grant USB Access Without Elevated Privileges
```bash
sudo usermod -a -G dialout $USER
```

11. Build the Workspace
```bash
cd ArthroBot/arthrobot_ws/
colcon build
```


### versions
- V2: Basic version



![facebook_cover_photo_1](https://github.com/Fadi-Eid/ArthroBot/assets/113466842/6c04bcef-7469-4d71-94f2-315197fd8f65)
