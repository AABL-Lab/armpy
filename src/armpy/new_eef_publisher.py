#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from moveit_commander import MoveGroupCommander

def main():
    rospy.init_node('eef_position_publisher')
    
    # Create a MoveGroupCommander object for the desired planning group
    group_name = "arm"  # Change this to your planning group name
    group = MoveGroupCommander(group_name)
    group.set_pose_reference_frame('base_link')
    group.set_end_effector_link('j2s7s300_ee_link')
    
    # Create the publisher
    eef_position_pub = rospy.Publisher('/eef_position', PointStamped, queue_size=10)
    
    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Get the current end-effector pose
        eef_pose = group.get_current_pose().pose
        
        # Create a PointStamped message
        eef_position_msg = PointStamped()
        eef_position_msg.header.stamp = rospy.Time.now()
        eef_position_msg.header.frame_id = group.get_planning_frame()
        eef_position_msg.point = eef_pose.position
        
        # Publish the message
        eef_position_pub.publish(eef_position_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
