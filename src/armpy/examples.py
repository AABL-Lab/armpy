#!/usr/bin/env python3

import math
import rospy
from arm_moveit2 import ArmMoveIt
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose

arm = ArmMoveIt(planning_frame="j2s6s300_link_base")
target = Pose()
target.position.x = 0.2
target.position.y = 0.2
target.position.z = 0.2
target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
    Transform.quaternion_from_euler(0.0, math.pi/2, math.pi/2)

arm.move_to_ee_pose(target)