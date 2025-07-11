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
rospy.init_node('record_position_joint_states')
arm = armpy.kortex_arm.Arm()
rospy.loginfo("Homing arm")
arm.home_arm()

rospy.loginfo("Done homing")
joint_state_data = rospy.wait_for_message('/my_gen3_lite/joint_states', JointState, timeout=.1).position
print(joint_state_data)
joint_state = copy.deepcopy(list(joint_state_data[5:12]))
print(joint_state) 
joint_state[1] -= 0.1
#arm.move_to_pose(joint_state_data, wait=True)
#arm.execute_action(joint_state, wait=True)

# (x, y, z) (qx, qy, qz, qw) rotation
default_pose = [0.5, 0, 0.05, 1, 1.0, 0.0, 0.0]
result = arm.goto_eef_pose(default_pose)

result = arm.close_gripper()
result = arm.open_gripper()
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