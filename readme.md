# Husky - UR3 manipulator
 
## Overview
The Husky UR3 mobile manipulation tutorial will show you how to operate a mobile manipulation robot using Gazebo, RViz, MoveIt, and the UR3 arm. 

There will be full control and actuation of the robot, and the software written for this robot can be translated into real world actions with the real Husky UR3 robot.


**Author: [Hyeonjun Park](https://www.linkedin.com/in/hyeonjun-park-41bb59125), koreaphj91@gmail.com**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**

## ROS Kinetic version

## Installation
- Before do this, please backup important files.

### Dependencies

This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

One line install: https://cafe.naver.com/openrt/14575 
```
for Desktop

wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

Additionally, LMS1xx for ROS depends on following software([LMS1xx ROS wiki](http://wiki.ros.org/LMS1xx))
```
sudo apt-get install ros-kinetic-lms1xx
```
- For contorling unversal robot with Moveit! python interface.
[ur3-moveit-test](https://github.com/MrLacquer/ur3-moveit-test.git)
$ cd ~/catkin_ws/src && git clone https://github.com/MrLacquer/ur3-moveit-test.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile

## How to start?

```
$ cd ~/catkin_ws/src && git clone https://github.com/MrLacquer/husky_ur3_manipluator.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile

- Bring up Gazebo and the Husky, driving the husky base using keyboard:
$ roslaunch husky_ur3_gazebo husky_ur3.launch
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

- Bring Up MoveIt & RViz
$ roslaunch husky_ur3_moveit_config husky_ur3_hj_moveit_planning_excution.launch sim:=true
$ roslaunch husky_ur3_moveit_config moveit_rviz.launch

- Contorlling the UR3 manipulator (more detail [ur3-moveit-test](https://github.com/MrLacquer/ur3-moveit-test.git))
$ rosrun ur3_moveit ur3_move.py 
or
$ rosrun ur3_moveit ur3_demo.py 
```

## Description
<img width="1000" src="https://user-images.githubusercontent.com/4105524/77276506-3848a780-6cfe-11ea-9672-eda8e562189b.png"  alt="Screenshot" title="Screenshot">


## Demo



