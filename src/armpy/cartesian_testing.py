#!/usr/bin/env python3

import math, time
import rospy
import numpy as np
from continuous_cartesian import go_to_relative
#from hlpr_manipulation.hlpr_manipulation_utils.src.hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
#import hlpr_manipulation.hlpr_manipulation_utils.src.hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose

try: 
    rospy.init_node("cartesian_testing", disable_signals=True)
except:
    pass

go_to_relative([.1,0,0,0,0,0], complete_action=True)
go_to_relative([-.1,0,0,0,0,0], complete_action=True)

# for i in range(100):
#     for q in range(10):
#         go_to_relative([.02,0,0,0,0,0], complete_action=False)
#         time.sleep(0.01)
#     for q in range(10):
#         go_to_relative([-.02,0,0,0,0,0], complete_action=False)
#         time.sleep(0.01)
# for i in range(20):
#     go_to_relative([0.05,0,0,0,0,0], complete_action=False)