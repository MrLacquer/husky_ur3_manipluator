#!/usr/bin/python

import roslaunch
import rospy
import socket
import time

ip = "ur3"
port = 30003
retry = 5
delay = 10
timeout = 3

def isOpen(ip, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
                s.connect((ip, int(port)))
                s.shutdown(socket.SHUT_RDWR)
                return True
        except:
                return False
        finally:
                s.close()

def checkHost(ip, port):
        ipup = False
        for i in range(retry):
                if isOpen(ip, port):
                        ipup = True
                        break
                else:
                        time.sleep(delay)
        return ipup


rospy.init_node('launching_ur3_ros_driver', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/husky/catkin_ws/src/husky_ur3_robot/husky_ur3_bringup/launch/ur3.launch"])


while not rospy.is_shutdown():
   connected= checkHost(ip,port)
   if connected:
         break

if connected: 
        rospy.sleep(50)
        launch.start()  

rospy.spin()
