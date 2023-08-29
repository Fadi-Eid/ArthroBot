# ArthroBot - 5DOF Desktop Robotic Arm

## Description
ArthroBot is a small general purpose 5 DOF desktop robotic arm. It is powered by ROS2 and MoveIt.
ArthroBot takes advantage of the latest advancements in the field of robotics and robotics manipulators to compute its kinematics, generate a collision-free path and ensure precise control of the end effector (gripper).

## Objective
ArthroBot is a prototype for a small, vesatile and general purpose programmable robotic arm that can be used in all areas and field such as:
* Educational: Universities and schools can use ArthroBot in the labs for practical robotics courses where kinematics and motion control algorithms and logic can be tested and developped.
* Personal: Individuals can make use of ArthroBot in office or at home to complete repetitive pre-programmed tasks or intelligent tasks when combined with computer vision and sensor fusion.
* Industrial: Industries such as photography, cinematography, security, defense, food and others can make use of ArthroBot to complete programmed or intelligent tasks such as anti-theft, video recording, tracking and surveillance, combat aid...

### Technical information
### Software
ArthroBot uses ROS2 Humble combined with MoveIt2 and other software packages to make the kinematics and motion computations and for teleoperation and simulation.
ArthroBot also includes in its software package a digital twin that can be used to simulate the robotic arm for testing without the use of the physical robot and hardware, thus speeding up development and testing.

### Hardware
ArthrBot makes use of an arduino UNO R3 for control. This decision was made because of the popularity and availability of the microcontroller as well as the simplicity of its programming which makes integration easier and development of the control much faster.
The microcontroller control 5 servo motors for the arm and the end effecor which is pen like and may be later substituted with any gripper.

### versions
- V1: Prototype version

![arthrobot](https://github.com/Fadi-Eid/ArthroBot/assets/113466842/8963e0f0-5329-40b9-9250-b5a21c0a6150)
