# Araco - Hexapod
<img width="1452" height="602" alt="araco - assembly v24(1) (1)" src="https://github.com/user-attachments/assets/35a75d7c-4a59-4b06-9095-5b99abdd390b" />


**Araco is a robotics passion project started in late 2022 and currently on its third generation. This repo is mainly to document the very whole process of building this hexapod, including designing, 3d-printing, coding, hardware, and further integration with depth vision, SLAM, Nav2 and potentially reinforcement learning using Isaac Lab.**

_The aim of this is for education and research only. I belive this documentation will be sufficient for anyone to build their own hexapod from scratch._

# Chapters


## Prereq information

### Part List
  - Raspberry Pi 5B
  - DS3235 35kg Servo _x18_
  - DS5160 60kg Servo _x6_
  - Hiwonder 32 Channel Digital Servo Controller
  - ORBBEC Gemini 335
  - Raspberry Pi Camera Module 3
  - PiSugar 3 Plus
  - 7.4V 7200mah lithium battery

### Development platform
  - ROS2 Jazzy
  - Ubuntu 24.04 running ROS2 from my laptop
  - Raspberry Pi OS running ROS2 docker image on the Pi
  - Isaac Sim 4.5.0

## Design

## Algorithms
### 3 DoF Inverse Kinematics
In robotics, inverse kinematics (IK) is the process of calculating the joint parameters required for a robot manipulator to place its end-effector (in this case its the tip of the legs) at a desired position and orientation in space.

Starts simple, take a leg with **3 DoF** as an example.

Set (x,y,z) as the target coordinates of our end effector. We are trying to find the position of the three joints denoted by θ1, θ2, and θ3. This can be solved by simple trigonometry.

First we look at the xy plane (top down view):

<img width="800" height="600" alt="IK1" src="https://github.com/user-attachments/assets/46baad9b-4b4d-4fe0-ac35-3d3156a4952d" />

θ1 can be easily solved by atan2. 



Now for the plane along all the joints:

<img width="800" height="600" alt="IK2" src="https://github.com/user-attachments/assets/511cc7ba-7dbc-447d-a88d-bb66f701024c" />

θ2  =  φ2 - φ4  =  φ2 - (π/2 - φ3)  , where we use the laws of cosines to find φ2 and use the trigonometric ratios to find φ3.

θ2  =  π - φ5  , where we can again use laws of cosines to find φ5

### 4 DoF Inverse Kinematics
**Araco adopts a 4 DoF configuration for each leg, and I want L4 to always be vertical to the ground in this plane, no matter the orientation of the body.**

<img width="800" height="600" alt="4DoF drawio" src="https://github.com/user-attachments/assets/f9c63ff8-51ec-4c15-878a-408e7abe8133" />

**This become a bit trickier, but in simple terms,**
  1. set up a unit vector
  2. apply the same tranformations (rotations) from the body on this unit vector
  3. set up the plane along all the joints
  4. project the unit vector onto the plane to get a new vector
  5. scale the vector by L4 and find the coordinates of the end effector joint
  6. use the 3 DoF IK to solve the rest of the joints
  7. lastly solve for the end effector joint

_(might write a detail solution for this in the future but this is my drawing notes for coding this)_
<img width="414" height="245" alt="Screenshot 2026-03-02 114028" src="https://github.com/user-attachments/assets/a41a8386-3501-475b-a06b-2ddd6b1a51eb" />
<img width="372" height="275" alt="Screenshot 2026-03-02 114049" src="https://github.com/user-attachments/assets/a8fa6966-b2d2-4d10-a11c-0af053705282" />

### Gait Control
Gait is the pattern of movement of the limbs of animals, including humans, during locomotion over a solid substrate. In order to let the robot walk, we need to devise a gait algorithm to control the sequence of movement for each of the 6 end effectors. 

Araco adopts the most common gait for hexapods - the tripod gait, for its speed, simplicity, and stability. Other common gaits include the ripple gait, tetrapod gait and the wave gait.        _The tripod gait draws a lot of power and torque as it only has 3 supporting legs at any point of time._
<img width="478" height="422" alt="applsci-11-01339-g001" src="https://github.com/user-attachments/assets/d1b4b857-0500-4f2d-beb9-4b7a30e08241" />
_Source: Luneckas, M., Luneckas, T., Kriaučiūnas, J., Udris, D., Plonis, D., Damaševičius, R., & Maskeliūnas, R. (2021). Hexapod Robot Gait Switching for Energy Consumption and Cost of Transport Management Using Heuristic Algorithms. Applied Sciences, 11(3), 1339. https://doi.org/10.3390/app11031339_




### Movement


## Isaac Sim

## Hardware

