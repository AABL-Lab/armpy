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
import armpy.arm
from sensor_msgs.msg import JointState
import copy
rospy.init_node('record_position_joint_states')
arm = armpy.arm.Arm()
#arm.home_arm()
joint_state_data = rospy.wait_for_message('joint_states', JointState, timeout=.1).position
print(joint_state_data)
joint_state = copy.deepcopy(list(joint_state_data[5:12]))
print(joint_state) 
joint_state[1] -= 0.1
#arm.move_to_pose(joint_state_data, wait=True)
arm.group.go(joint_state, wait=True)