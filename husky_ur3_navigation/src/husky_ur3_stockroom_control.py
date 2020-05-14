#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys, time
import rospy
import copy, math
import tf
import moveit_msgs.msg
from math import pi

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import *
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Empty

GROUP_NAME_ARM = "arm"
FIXED_FRAME = 'map'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class manipulator_control():
      def __init__(self):
            roscpp_initialize(sys.argv)        
            rospy.init_node('husky_ur3_stockroom_control_node',anonymous=True)

            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                      moveit_msgs.msg.DisplayTrajectory,
                                                      queue_size=20)
                        
            self.goal_x = 0
            self.goal_y = 0
            self.goal_z = 0
            self.goal_roll = 0
            self.goal_pitch = 0
            self.goal_yaw = 0

            self.goal_ori_x = 0
            self.goal_ori_y = 0
            self.goal_ori_z = 0
            self.goal_ori_w = 0

            self.marker = []
            self.position_list = []
            self.orientation_list = []

            self.m_idd = 0
            self.m_pose_x = []
            self.m_pose_y = []
            self.m_pose_z = []
            self.m_ori_w = []
            self.m_ori_x = []
            self.m_ori_y = []
            self.m_ori_z = []

            self.ar_pose = Pose()
            self.goalPoseFromAR = Pose()
            self.br = tf.TransformBroadcaster()

            self.trans = []
            self.rot = []

            self.target_ar_id = 10

            self.calculed_coke_pose = Pose()
            self.pose_goal = Pose()
            #rospy.loginfo("Waiting for ar_pose_marker topic...")
            #rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            #rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
            #rospy.loginfo("Maker messages detected. Starting followers...")

            #self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

            self.scene = PlanningSceneInterface()
            self.robot_cmd = RobotCommander()

            self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
            #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
            self.robot_arm.set_goal_orientation_tolerance(0.005)
            self.robot_arm.set_planning_time(5)
            self.robot_arm.set_num_planning_attempts(5)

            self.display_trajectory_publisher = display_trajectory_publisher

            rospy.sleep(2)
            # Allow replanning to increase the odds of a solution
            self.robot_arm.allow_replanning(True)                 
            
            #self.clear_octomap()
            
      def ar_pose_subscriber(self):
            rospy.loginfo("Waiting for ar_pose_marker topic...")
            rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_tf_listener)
            rospy.loginfo("Maker messages detected. Starting followers...")

            print "======= pos(meter): ", self.position_list
            print "======= orientation: ", self.orientation_list  

      def ar_tf_listener(self, msg):
            try:
                  self.marker = msg.markers
                  
                  ml = len(self.marker)
                  target_start_point_id = self.target_ar_id
                  #target_id = target_ar_id
                  #self.m_idd = self.marker[0].id  # 임시용

                  for ii in range(0, ml): # 0 <= ii < ml
                        self.m_idd = self.marker[ii].id
                        #print "checked all id: ", self.m_idd
                        if self.m_idd != target_start_point_id:
                              pass
                              #target_id_flage = False
                        elif self.m_idd == target_start_point_id:
                              target_id_flage = True
                              target_id = self.m_idd
                              target_id_index = ii

                  #print "target id: ", target_id_index, target_id, target_id_flage
                  
                  #
                  # target ar marker의 위치 저장 부분
                  #
                  if target_id_flage == True:
                        self.ar_pose.position.x = self.marker[target_id_index].pose.pose.position.x
                        self.ar_pose.position.y = self.marker[target_id_index].pose.pose.position.y
                        self.ar_pose.position.z = self.marker[target_id_index].pose.pose.position.z
                        self.ar_pose.orientation.x = self.marker[target_id_index].pose.pose.orientation.x
                        self.ar_pose.orientation.y = self.marker[target_id_index].pose.pose.orientation.y
                        self.ar_pose.orientation.z = self.marker[target_id_index].pose.pose.orientation.z
                        self.ar_pose.orientation.w = self.marker[target_id_index].pose.pose.orientation.w
                  
                  self.goal_x = self.ar_pose.position.x
                  self.goal_y = self.ar_pose.position.y
                  self.goal_z = self.ar_pose.position.z

                  self.position_list = [self.goal_x, self.goal_y, self.goal_z]
                  self.orientation_list = [self.ar_pose.orientation.x, self.ar_pose.orientation.y, self.ar_pose.orientation.z, self.ar_pose.orientation.w]
                  (self.goal_roll, self.goal_pitch, self.goal_yaw) = euler_from_quaternion(self.orientation_list) #list form으로 넘겨주어야 함
                  #print "======= pos(meter): ", self.goal_x, self.goal_y, self.goal_z
                  #print "======= rot(rad): ", self.goal_roll, self.goal_pitch, self.goal_yaw                 
                  #print "ar_pos(meter): \n", self.position_list
                  #print "ar_orientation: \n", self.orientation_list    
                  
            except:
                  return

      def move_moveit_setting_pose(self, pose_name):
            planning_frame = self.robot_arm.get_planning_frame()
            print "========== plannig frame: ", planning_frame

            self.wpose = self.robot_arm.get_current_pose()
            print"====== current pose : ", self.wpose                 

            if pose_name == "home":
                  self.robot_arm.set_named_target("home")
            elif pose_name == "zeros":
                  self.robot_arm.set_named_target("zeros")
            elif pose_name == "table":
                  self.robot_arm.set_named_target("table")
                  
            #print "Press the Enter"
            #raw_input()
            self.robot_arm.go(wait=True)


if __name__ == '__main__':
      shoulder_pan_joint = 0
      shoulder_lift_joint = 1
      elbow_joint = 2
      wrist_1_joint = 3
      wrist_2_joint = 4
      wrist_3_joint = 5

      mc = manipulator_control()

      rate = rospy.Rate(10.0)

      setting_pose = "home"
      mc.move_moveit_setting_pose(setting_pose)

      mc.target_ar_id = 10
      mc.ar_pose_subscriber()
      time.sleep(0.5)