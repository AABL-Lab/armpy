#!/usr/bin/env python3

import moveit_commander
import rospy
import sys

import armpy.arm
import armpy.gripper

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_test', anonymous=True)

    arm = armpy.arm.Arm()
    gripper = armpy.gripper.Gripper()
    import IPython; IPython.embed()

if __name__ == "__main__":
    main()
