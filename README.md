# Araco-Hexapod
Araco is a robotics passion project started in late 2022 and currently on its third generation. This repo is mainly to document the very whole process of building this hexapod, including designing, 3d-printing, coding, hardware, and further integration with depth vision, SLAM, Nav2 and potentially reinforcement learning using Isaac Lab.

The ROS2 package (the code), STL files, Isaac Sim stage will only be released when this generation is consider completed, however, I belive this documentation will be sufficient for anyone to build their own hexapod from scratch.

*I use a little bit of ai for the sake of writing.

## Prereq information

### Part List
  - Raspberry Pi 5B
  - DS3235 35kg Servo _x18_
  - DS5160 60kg Servo _x6_
  - Hiwonder 32 Channel Digital Servo Controller
  - ORBBEC Gemini 335
  - 7.4V 7200mah lithium battery

### Development platform
  - ROS2 Jazzy
  - Ubuntu 24.04 running ROS2 from my laptop
  - Raspberry Pi OS running ROS2 docker image on the Pi
  - Isaac Sim 4.5.0

## Design

## Algorithms
### Inverse Kinematics
In robotics, inverse kinematics (IK) is the process of calculating the joint parameters required for a robot manipulator to place its end-effector (in this case its the tip of the legs) at a desired position and orientation in space. There are many way to solve IK but for the sake of simplicity and interpretability I will start with the geometric approach.






### Gait Control

### Movement


## Isaac Sim

## Hardware

