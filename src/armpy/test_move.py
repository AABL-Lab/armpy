#!/usr/bin/env python3

import os
import cv2
import math
import rospy
import tf2_ros
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
#from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Image as msg_Image
import armpy.kortex_arm
from sensor_msgs.msg import JointState
import copy
import time
rospy.init_node('record_position_joint_states')
arm = armpy.kortex_arm.Arm()
rospy.loginfo("Homing arm")
arm.home_arm()

rospy.loginfo("Done homing")
########### subscribe to depth camera node #########

# joint_state_data = rospy.wait_for_message('/my_gen3_lite/joint_states', JointState, timeout=.1).position
# print("All joint state data", joint_state_data)
# joint_state = copy.deepcopy(list(joint_state_data[5:12]))
# #print("Joint states", joint_state) 
# joint_state[1] -= 0.1
# #arm.move_to_pose(joint_state_data, wait=True)
# #arm.execute_action(joint_state, wait=True)

# # (x, y, z) (qx, qy, qz, qw) rotation
# # qy: moves gripper joint :)
# x_center = 0.5
# # default_pose = [0.5, 0, 0.0, 1, 1.0, 0.0, 0.0]
# default_pose = [0.5, 0, 0.5, 1.0, 0.0, 0.0, 0.0]
# print("moving to default pose")
# result = arm.goto_eef_pose(default_pose)
# time.sleep(5)

# joint_state_data = rospy.wait_for_message('/my_gen3_lite/joint_states', JointState, timeout=.1).position
# print("All joint state data", joint_state_data)

# # Pass values from genesis simulation to real robot
# arm_position_dofs = [ 0.24357289, -1.3126478, -0.23887946, -0.14974634, 1.7751877, 0.9315492, 0.00602896, -0.00302473, -0.003478, -0.00360355]
# arm_waypoints = arm_position_dofs[:6]
# gripper_waypoints = arm_position_dofs[6:]
# print(arm_waypoints)
# print("Going to joint pose")
# results = arm.goto_joint_pose(arm_waypoints)
# time.sleep(5)
# arm_position2 = [-9.4415337e-02, -1.6932747e+00, -4.7102508e-01,  1.5638263e+00,
#   2.0483732e+00,  1.4763145e+00, -6.3851485e-03, -2.4709428e-02,
#   7.4626954e-04, -9.4122291e-03]

# arm_waypoints = arm_position2[:6]
# print(arm_waypoints)
# results = arm.goto_joint_pose(arm_waypoints)
# #results = arm.goto_joint_gripper_waypoints(gripper_waypoints)
# # print("closing gripper")
# result = arm.close_gripper()
# time.sleep(2)
# print("moving to next pose")
# next_pose = [x_center, 0, 0.05, 1, 1.0, 0.0, 0.0]
# result = arm.goto_eef_pose(next_pose)
# time.sleep(2)
# result = arm.open_gripper()
# time.sleep(2)
# radius = 0.1

# print("tilting gripper ")
# next_pose = [x_center, 0, 0.05, 1, 0.0, 0.0, 0.0]
# result = arm.goto_eef_pose(next_pose)
# time.sleep(2)
# result = arm.open_gripper()
# time.sleep(2)


# print("moving to clay edge")
# move_to_edge = [x_center, 0 + radius, 0.05, 1, 1.0, 0.0, 0.0]
# result = arm.goto_eef_pose(move_to_edge)
# step_size = 0.01
# angle_radians = math.pi / 4
# time.sleep(10)
# print("moving along circle arc")
# x_new = x_center + np.cos(angle_radians) * radius
# y_new = np.sin(angle_radians) * radius
# circle_path = [x_new, y_new, 0.05, 1, 1.0, 0.0, 0.0]
# result = arm.goto_eef_pose(circle_path)
# time.sleep(3)

# x_new = x_new + np.cos(angle_radians) * radius
# y_new = y_new np.sin(angle_radians) * radius
# circle_path = [x_new, y_new, 0.05, 1, 1.0, 0.0, 0.0]
# result = arm.open_gripper()

# Policy: all states map to same action

# control interface to record and replay demonstrations
# replicate behaviors 
# robot and clay start at same position
# collect and replay demonstration
# repo for collecting and replaying demonstrations from teleop
# build something simple in genesis
# look for other soft objects in genesis then map same setup in real world
# convert depth into cartesian positions
# 

# 1) implement teleoperation
# 2) demonstration collection
# 3) qr code

# train to poke an obect 
# customize what the learning agent has learned
# make cup or  bowl to be deeper or walls to be thinner
# learn how to choose different policy
# reach forward then moving forward or backward
# make demonstration consistent for behavior cloning
# diffusion policy
#   diffusion based behavior cloning
# read paper on diffusion policy
# read summary papers
# dont focus too much on learning//behavior cloning
# for paper explain execution of 1-2 things