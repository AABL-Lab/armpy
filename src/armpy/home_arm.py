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
joint_state_data = rospy.wait_for_message('/my_gen3_lite/joint_states', JointState, timeout=.1).position