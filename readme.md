# Husky - UR3 manipulator
 
## Overview
The Husky UR3 mobile manipulation tutorial will show you how to operate a mobile manipulation robot using Gazebo, RViz, MoveIt, and the UR3 arm. 

There will be full control and actuation of the robot, and the software written for this robot can be translated into real world actions with the real Husky UR3 robot.



**Author:**   
- **Kinetic version - [Hyeonjun Park](https://www.linkedin.com/in/hyeonjun-park-41bb59125), koreaphj91@gmail.com**  
- **Melodic version - [Sang-heum Lee](https://github.com/Shumine), heumine@khu.ac.kr**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**

## This pakage for ROS Melodic version

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
```
$ cd ~/catkin_ws/src && git clone https://github.com/MrLacquer/ur3-moveit-test.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

- For hector slam  
[hector-slam](http://wiki.ros.org/hector_slam)  
```
$ cd ~/catkin_ws/src && git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
*if you're Kinetic version,*
$ cd ~/catkin_ws/src/hector_slam
$ git checkout catkin
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

- For husky mobile robot control  
[four-wheel-steer-teleop](https://github.com/gkouros/four-wheel-steer-teleop.git)  
```
$ cd ~/catkin_ws/src && git clone https://github.com/gkouros/four-wheel-steer-teleop.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
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
$ roslaunch husky_ur3_test_moveit_config husky_ur3_planning_excution.launch
$ roslaunch husky_ur3_moveit_config moveit_rviz.launch

- Contorlling the UR3 manipulator (more detail [ur3-moveit-test](https://github.com/MrLacquer/ur3-moveit-test.git))
$ rosrun ur3_moveit ur3_move.py 
or
$ rosrun ur3_moveit ur3_demo.py 
```

- Husky UR3 manipulator with hector slam
```
$ roslaunch husky_ur3_gazebo husky_ur3.launch
*(Optional)* $ roslaunch husky_ur3_test_moveit_config husky_ur3_planning_excution.launch
$ roslaunch husky_ur3_moveit_config moveit_rviz.launch
$ roslaunch husky_ur3_navigation husky_ur3_mapping.launch  
$ rosrun four_wheel_steer_teleop four_wheel_steer_teleop.py 


To save the generated map, you can run the map_saver utility:
$ rosrun map_server map_saver -f <filename>
```

- Husky UR3 manipulator naviagation 
```
$ roslaunch husky_ur3_gazebo husky_ur3.launch
*(Optional)* $ roslaunch husky_ur3_test_moveit_config husky_ur3_planning_excution.launch
$ roslaunch husky_ur3_moveit_config moveit_rviz.launch
$ roslaunch husky_ur3_navigation husky_ur3_amcl.launch
```

## Note
- ~~2020.03.25: husky robot's IMU data is missing. We're trying to fixing this problems.~~  
   -> It has been fixed. The problem is '[control.yaml](https://github.com/MrLacquer/husky_ur3_manipluator/blob/master/husky_ur3_gazebo/config/control.yaml)' file.  
      Thank you! [Sang-heum Lee](https://github.com/Shumine)

- 'param' folder is for Turtlebot3's AMCL. Not used.
```
amcl_test_code.launch
move_base_test_code.launch
```



## Description


## Demo
### Launch the Gazebo and Moveit!
<img width="1000" src="https://user-images.githubusercontent.com/4105524/77276506-3848a780-6cfe-11ea-9672-eda8e562189b.png"  alt="Screenshot" title="Screenshot">

### Launch the Gazebo and Navigation 
<img width="1000" src="https://user-images.githubusercontent.com/4105524/77393465-6a323a80-6de0-11ea-9a34-82e4ebc3fe79.png"  alt="Screenshot" title="Screenshot">
