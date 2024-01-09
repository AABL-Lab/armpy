#!/usr/bin/env python3

import rospy
import armpy

def main():
    rospy.init_node("test_generic", anonymous=True)

    robot = armpy.initialize("gen3_lite")

    robot.home_arm()
    import IPython
    IPython.embed()

if __name__ == "__main__":
    main()