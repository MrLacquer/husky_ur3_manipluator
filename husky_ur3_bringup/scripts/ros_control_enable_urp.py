#!/usr/bin/env python

"""

Software License Agreement (BSD)
file      ros_control_enable_urp.py
authors   Zakaria Chekakta <zakaria.chekakta@gaitech.co.kr>
copyright Copyright (c) 2020, Gaitech Korea, All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import sys
import rospy
import time
from std_srvs.srv import Trigger


import actionlib
from ur_dashboard_msgs.msg import * 
from ur_dashboard_msgs.srv import Load , Popup , GetRobotMode, GetLoadedProgram, GetProgramState

def load(filename):
    rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
    
    try:
        load_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        res=load_program(filename)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)

def play():
    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    try:
        play_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        resp1 = play_program()
        return resp1.message
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)


def power_and_start():
    
    client = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', ur_dashboard_msgs.msg.SetModeAction)
    client.wait_for_server()
    goal = ur_dashboard_msgs.msg.SetModeGoal(target_robot_mode=ur_dashboard_msgs.msg.RobotMode.RUNNING)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()  

def message(msg):
    rospy.wait_for_service('/ur_hardware_interface/dashboard/popup')
    try:
        message_to_display = rospy.ServiceProxy('/ur_hardware_interface/dashboard/popup', Popup)
        resp1 = message_to_display(msg)
        return resp1.answer
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)

def main ():
    rospy.init_node('Allowing_ros_control')
    filename=rospy.get_param('ros_control_enable_urp', '/programs/ros_contol_enable.urp')
    enabling_robot_msg="Powering on and enabling the external control of the UR3 robot, please wait!"
    robot_state_msg="UR3 Robot ready to receive control commands."

    rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode')
    robot_mode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)

    rospy.wait_for_service('/ur_hardware_interface/dashboard/get_loaded_program')
    loaded_program = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
    
    rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
    program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
    
    rospy.wait_for_service('/ur_hardware_interface/dashboard/close_popup')
    close_msg_window = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_popup', Trigger)

    message(enabling_robot_msg)
    time.sleep(6)
    close_msg_window()


    counter=0

    while counter < 10 :    
        counter +=1
        if robot_mode().robot_mode != ur_dashboard_msgs.msg.RobotMode.RUNNING :
            power_and_start()
            robot_mode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)    
        if loaded_program().program_name != filename:
            load(filename)
       
    
    if program_state().state != ur_dashboard_msgs.msg.ProgramState.PLAYING:
            play()

    message(robot_state_msg)
    time.sleep(6)
    close_msg_window()
    text=robot_mode().answer
    print( text )


if __name__ == '__main__':
    sys.exit(main())    