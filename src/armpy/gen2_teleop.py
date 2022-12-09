#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from kinova_msgs.msg import PoseVelocity
from kinova_msgs.srv import Start, Stop


DEFAULT_ROBOT_NS = "gen2"
CARTESIAN_VEL_TOPIC = "/in/cartesian_velocity"
START_SERVICE = "/in/start"
STOP_SERVICE = "/in/stop"

class Gen2Teleop:
    def __init__(self, ns=None):
        if ns is None:
            ns = rospy.resolve_name(DEFAULT_ROBOT_NS)
        self._cart_vel_pub = rospy.Publisher(ns + CARTESIAN_VEL_TOPIC, PoseVelocity, queue_size=1)
        self._start = rospy.ServiceProxy(ns + START_SERVICE, Start)
        self._stop = rospy.ServiceProxy(ns + STOP_SERVICE, Stop)

        self._timer = None
        self._command = None
        self._started = False

    def _timer_cb(self, evt):
        self._cart_vel_pub.publish(self._command)

    def set_velocity(self, twist):
        if not self._started:
            self._start()
            self._started = True
        self._command = PoseVelocity(
                twist_linear_x = twist.linear.x,
                twist_linear_y = twist.linear.y,
                twist_linear_z = twist.linear.z,
                twist_angular_x = twist.angular.x,
                twist_angular_y = twist.angular.y,
                twist_angular_z = twist.angular.z
            )
        if not self._timer:
            self._timer = rospy.Timer(rospy.Duration(0.01), self._timer_cb, oneshot=False)

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

    import IPython; IPython.embed()
