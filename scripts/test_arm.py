#!/usr/bin/env python

import moveit_commander
import rospy
import sys

import armpy.arm

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_test', anonymous=True)

    arm = armpy.arm.Arm()
    import IPython; IPython.embed()

if __name__ == "__main__":
    main()
