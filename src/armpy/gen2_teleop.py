#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from kinova_msgs.msg import PoseVelocity
from kinova_msgs.srv import Start, Stop, HomeArm
import numpy as np


DEFAULT_ROBOT_NS = "/j2s7s300_driver"
CARTESIAN_VEL_TOPIC = "/in/cartesian_velocity"
START_SERVICE = "/in/start"
STOP_SERVICE = "/in/stop"
HOME_ARM_SERVICE = "/in/home_arm"

class Gen2Teleop:
    def __init__(self, ns=None, home_arm=True,collision_detection=False ):
        self._collision_detection = collision_detection
        if ns is None:
            ns = rospy.resolve_name(DEFAULT_ROBOT_NS)
        self._cart_vel_pub = rospy.Publisher(ns + CARTESIAN_VEL_TOPIC, PoseVelocity, queue_size=1)
        self._start = rospy.ServiceProxy(ns + START_SERVICE, Start)
        self._stop = rospy.ServiceProxy(ns + STOP_SERVICE, Stop)

        self._timer = None
        self._command = None
        # home the arm so that cartesian velocity control works
        if home_arm:
            home_arm_now = rospy.ServiceProxy(ns+HOME_ARM_SERVICE, HomeArm)
            rospy.logwarn("press any key to home arm and enable cartesian control")
            input()
            rospy.logwarn("Homing Arm for cartesian control")
            home_arm_now()

        ####prepare for collision checking
        if collision_detection:
            # subscribe to joint joint states
            rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
            # prepare service for collision check
            self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
            # wait for service to become available
            self.sv_srv.wait_for_service()
            rospy.loginfo('service is avaiable')
            # prepare msg to interface with moveit
            self.rs = RobotState()
            self.rs.joint_state.name = ["j2s7s300_joint_1","j2s7s300_joint_2",
                                    "j2s7s300_joint_3","j2s7s300_joint_4",
                                    "j2s7s300_joint_5","j2s7s300_joint_6",
                                    "j2s7s300_joint_7"]
            self.rs.joint_state.position = [0.0, 0.0,0.0,0.0,0.0,0.0,0.0]
            self.joint_states_received = False

        self._started = True

    def _timer_cb(self, evt):
        '''
        At each timer interval, publish the PoseVelocity command 
        from set_velocity to ROS
        '''
        self._cart_vel_pub.publish(self._command)

    def set_velocity(self, twist):
        """
        Attributes
        ----------
        twist : geometry_msgs.msg Twist
            The linear and angular velocity of controller
        """
        if not self._started:
            self._start()
            self._started = True
            
        if  self._collision_detection:
            twist = self.detect_collisions(twist)
            
        self._command = PoseVelocity(
            twist_linear_x = twist.linear.x,
            twist_linear_y = twist.linear.y,
            twist_linear_z = twist.linear.z,
            twist_angular_x = twist.angular.x,
            twist_angular_y = twist.angular.y,
            twist_angular_z = twist.angular.z
            )

        if not self._timer:
            # Timer must be 0.1 because Gen2 requires velocity commands at 100Hz
            self._timer = rospy.Timer(rospy.Duration(0.01), self._timer_cb, oneshot=False)

    def detect_collision(twist):
        '''
        If the current twist being sent is 0, don't bother checking 
        for collisions. Otherwise, use the requested velocity to determine 
        the future position, and check there for collisions.
        If a collision is detected, all the twist components are set to 0.
        '''

        if np.isclose([twist.linear.x, twist.linear.y, twist.linear.z,
                       twist.angular.x, twist.angular.y, twist.angular.z],
                      [0.0,0.0,0.0,0.0,0.0,0.0])
            newtwist=twist
            return newtwist
        else:
            # get joint position
            # using jacobian get new joint positions after (time - .1 or longer?)
            # check for collisions with moveit getstatevalidity on new position
            # if new state has collisions, set twists to 0 and throw error
            
    def stop(self):
        self._cart_vel_pub.publish()
        self._timer.shutdown()
        self._timer = None

    def force_stop(self):
        self._stop()
        self.stop()
        self._started = False

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)
    arm = Gen2Teleop()

    #import IPython; IPython.embed()
