#!/usr/bin/env python

import moveit_commander
import rospy
import sys

import armpy

def main():
    rospy.init_node('arm_test', anonymous=True)

    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        name = "gen3_lite"
    arm = armpy.initialize(name)
    import IPython; IPython.embed()

if __name__ == "__main__":
    main()
