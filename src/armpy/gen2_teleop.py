#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from kinova_msgs.msg import PoseVelocity


DEFAULT_ROBOT_NS = "/gen2"
CARTESIAN_VEL_TOPIC = "in/cartesian_velocity"

class Gen2Teleop:
    def __init__(self):
        self._cart_vel_pub = rospy.Publisher(DEFAULT_ROBOT_NS + CARTESIAN_VEL_TOPIC, PoseVelocity)

    def set_vel(self, twist):
        self._cart_vel_pub(
            PoseVelocity(
                twist_linear_x = twist.linear.x,
                twist_linear_y = twist.linear.y,
                twist_linear_z = twist.linear.z,
                twist_angular_x = twist.angular.x,
                twist_angular_y = twist.angular.y,
                twist_angular_z = twist.angular.z
            )
        )

    def stop(self):
        self._cart_vel_pub()

