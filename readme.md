# Husky - UR3 manipulator

## Overview
The Husky UR3 mobile manipulation tutorial will show you how to operate a mobile manipulation robot using Gazebo, RViz, MoveIt, and the UR3 arm. 

There will be full control and actuation of the robot, and the software written for this robot can be translated into real world actions with the real Husky UR3 robot.


**Author: [Hyeonjun Park](https://www.linkedin.com/in/hyeonjun-park-41bb59125), koreaphj91@gmail.com**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**



## Installation
- Before do this, please backup important files.

### Dependencies

This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

One line install: https://cafe.naver.com/openrt/14575 
```
for Desktop

wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

## How to start?

```
$ cd ~/catkin_ws/src && git clone https://github.com/MrLacquer/husky_ur3_manipluator.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile

- Bring up Gazebo and the Husky, driving the husky base using keyboard:
$ roslaunch husky_ur3_gazebo husky_ur3.launch
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

- Bring Up MoveIt & RViz
$ roslaunch husky_ur3_moveit_config demo.launch   
```

## Description


## Demo



