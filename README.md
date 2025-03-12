# ArthroBot - (5+1) DOF Desktop Robotic Arm

## Overview
Welcome to ArthroBot, an innovative 6-degree-of-freedom (5+1-DOF) desktop robotic arm powered by ROS 2 and MoveIt 2. ArthroBot leverages the latest advancements in robotics and manipulators science to compute kinematics, generate collision-free paths, and deliver real-time control over its end effector (which can be customized for various applications).

![image](https://github.com/user-attachments/assets/6aa054a5-7137-414c-8ad8-8b1e9e9f9500)

## Mission
ArthroBot sets out to make the world of robotics accessible with its compact, low-cost, and simple design. This robotic arm finds utility across a spectrum of domains:
* Educational: ArthroBot provides a valuable tool for education, enabling students at universities and schools to gain practical experience in robotics. It serves as a hands-on platform for experimenting with and developing kinematics, motion control algorithms, and logic. The robot model can be easily replaced with another with minimal effort.
* Personal: For individual users, ArthroBot becomes a dexterous tool. It excels in completing repetitive pre-programmed tasks or more complex operations when combined with computer vision and sensor fusion technologies.
* Industrial: In industrial settings, ArthroBot shines in diverse applications including photography, cinematography, security, defense, and food processing. It can be programmed to perform a wide range of programmed or intelligent tasks such as anti-theft surveillance, video recording, tracking, and surveillance, as well as combat support operations (with the help of the MoveIt 2 software suite).

### Technical Specification
### Software Stack
ArthroBot relies on a modular software stack comprising ROS 2 Humble and MoveIt 2, along with other complementary software packages for communication and control. This provides a ready-to-use framework for kinematics and motion computations, teleoperation, and simulation. Moreover, ArthroBot includes a digital twin for visualization, allowing developers to simulate (Visualize is a better term for this version) the robotic arm for testing and refinement without the need for the physical hardware, thus accelerating development and testing cycles. In its current version, ArthroBot automatically detects the presence of the hardware and determines which mode (simulation or real) must be started. In case the hardware is removed at runtime, ArthroBot will automatically switch to hardware and will signal this switch to the user.

### Hardware Components
ArthroBot is driven by an **Arduino UNO R3** microcontroller for precise control. The selection of this microcontroller is based on its widespread popularity and accessibility, as well as for the simplicity of its programming, which accelerates communication and control system development. The microcontroller manages the motion of five servo motors for the arm, and one servo motor for the gripper.

Explore ArthroBot and unlock a world of possibilities in robotics, automation, and beyond. Join us in the journey of innovation, exploration, and creation.




### versions
- V2: Basic version



![facebook_cover_photo_1](https://github.com/Fadi-Eid/ArthroBot/assets/113466842/6c04bcef-7469-4d71-94f2-315197fd8f65)
