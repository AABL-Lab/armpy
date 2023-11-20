#! /usr/bin/env python3
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
from rospy.client import wait_for_message
import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.srv
from tf.transformations import euler_from_quaternion
import math
import argparse
from kinova_msgs.srv import AddPoseToCartesianTrajectory, ClearTrajectories

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

def cartesian_pose_client(position, orientation, rad_pose, collision_check=False, wait_for_action=False):
    # if wait_for_action the robot will preform the action till completion the stop.
    # else the action will be sent to a buffer of actions 

    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
    cartesian_serv = rospy.ServiceProxy('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
    rospy.wait_for_service('/j2s7s300_driver/in/clear_trajectories')
    cartesian_clear = rospy.ServiceProxy('/j2s7s300_driver/in/clear_trajectories', ClearTrajectories)


    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    euler_angle = [orientation[0],orientation[1],orientation[2],orientation[3]]
    #goal_list = [position[0],position[1], position[2], euler_angle[0], euler_angle[1],euler_angle[2]]
    goal_list = rad_pose
    #print(euler_angle, "AAAAAA")
    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    if collision_check:
        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

        goal_stamped = geometry_msgs.msg.PoseStamped()
        goal_stamped.header = goal.pose.header
        goal_stamped.pose = goal.pose.pose

        msgs_request = moveit_msgs.msg.PositionIKRequest()
        msgs_request.group_name = "arm"
        msgs_request.pose_stamped = goal_stamped
        msgs_request.robot_state.is_diff = True
        msgs_request.timeout.secs = .4
        msgs_request.avoid_collisions = True
        msgs_request.ik_link_names = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4",
                        "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]

        # msgs_request.robot_state = self.robot.get_current_state()
        try:
            jointAngle=compute_ik(msgs_request)
            ans=list(jointAngle.solution.joint_state.position)[2:9]
            ans = simplify_joints(ans)
            if jointAngle.error_code.val == -31:
                print('No IK solution')
                return -31
            if (jointAngle.error_code.val == -12 or jointAngle.error_code.val==-10):
                print("Goal or current position is in collision")
                return -12
                
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    #cartesian_serv(0.144489221573,-0.462273001671, 0.40168389678,1.6176496614678213, -0.20928853117355628, 1.111220579362625)
    if wait_for_action:
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(2.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None 
    else:
        try:
            cartesian_serv(goal_list[0], goal_list[1], goal_list[2],goal_list[3],goal_list[4],goal_list[5])
            #cartesian_clear()
        except rospy.ServiceException as e:
            print("Cartesian service call failed: %s"%e)


def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_



def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    #print(prefix)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    #print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='mq',
                        choices={'mq', 'mdeg', 'mrad'},
                        help='Unit of Cartesian pose command, in mq(Position meter, Orientation Quaternion),  mdeg(Position meter, Orientation Euler-XYZ in degree), mrad(Position meter, Orientation Euler-XYZ in radian)]')
    parser.add_argument('pose_value', nargs='*', type=float, help='Cartesian pose values: first three values for position, and last three(unit mdeg or mrad)/four(unit mq) for Orientation')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display Cartesian pose values in alternative convention(mq, mdeg or mrad)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    # print('pose_mq in argumentParser 1: {}'.format(args_.pose_value))  # debug
    return args_


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, pose_value_, relative_):
    """ Argument unit """
    global currentCartesianCommand

    position_ = pose_value_[:3]
    orientation_ = pose_value_[3:]

    for i in range(0,3):
        if relative_:
            position_[i] = pose_value_[i] + currentCartesianCommand[i]
        else:
            position_[i] = pose_value_[i]

    # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

    if unit_ == 'mq':
        if relative_:
            orientation_XYZ = Quaternion2EulerXYZ(orientation_)
            orientation_xyz_list = [orientation_XYZ[i] + currentCartesianCommand[3+i] for i in range(0,3)]
            orientation_q = EulerXYZ2Quaternion(orientation_xyz_list)
        else:
            orientation_q = orientation_

        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))

    elif unit_ == 'mdeg':
        if relative_:
            orientation_deg_list = list(map(math.degrees, currentCartesianCommand[3:]))
            orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
        else:
            orientation_deg = orientation_

        orientation_rad = list(map(math.radians, orientation_deg))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    elif unit_ == 'mrad':
        if relative_:
            orientation_rad_list =  currentCartesianCommand[3:]
            orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
        else:
            orientation_rad = orientation_

        orientation_deg = list(map(math.degrees, orientation_rad))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)

    else:
        raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

    pose_mq_ = position_ + orientation_q
    pose_mdeg_ = position_ + orientation_deg
    pose_mrad_ = position_ + orientation_rad

    # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

    return pose_mq_, pose_mdeg_, pose_mrad_


def verboseParser(verbose, pose_mq_):
    """ Argument verbose """
    position_ = pose_mq_[:3]
    orientation_q = pose_mq_[3:]
    if verbose:
        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))
        """print('Cartesian position is: {}'.format(position_))
        print('Cartesian orientation in Quaternion is: ')
        print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]))
        print('Cartesian orientation in Euler-XYZ(radian) is: ')
        print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
        print('Cartesian orientation in Euler-XYZ(degree) is: ')
        print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))"""

def go_to_relative(pose, collision_check=False, complete_action=True):
    kinova_robotTypeParser("j2s7s300")
    try:
        rospy.init_node(prefix + 'pose_action_client')
    except:
        pass

    """if args.unit == 'mq':
        if len(args.pose_value) != 7:
            print('Number of input values {} is not equal to 7 (3 position + 4 Quaternion).'.format(len(args.pose_value)))
            sys.exit(0)
    elif (args.unit == 'mrad') | (args.unit == 'mdeg'):
        if len(args.pose_value) != 6:
            print('Number of input values {} is not equal to 6(3 position + 3 EulerAngles).'.format(len(args.pose_value)))
            sys.exit(0)
    else:
        raise Exception('Cartesian value have to be in unit: mq, mdeg or mrad')"""

    getcurrentCartesianCommand(prefix)

    #for i in range(300):
        #pose = [-0.01, 0.001, 0.0, 0.0, 0.0, 0.0]

    pose_mq, pose_mdeg, pose_mrad = unitParser('mdeg', pose, True)
    #print(pose_mrad)
    try:

        poses = [float(n) for n in pose_mq]

        result = cartesian_pose_client(poses[:3], poses[3:], rad_pose=pose_mrad, collision_check=collision_check, wait_for_action=complete_action)

        #print('Cartesian pose sent!')

    except rospy.ROSInterruptException:
        print("program interrupted before completion")


    verboseParser(True, poses)
    return result

def simplify_angle(angle):
    # Very simple function that makes sure the angles are between -pi and pi
    if angle > math.pi:
        while angle > math.pi:
            angle -= 2*math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2*math.pi

    return angle

def simplify_joints(joints, group_id=0):
        # Helper function to convert a dictionary of joint values
        if isinstance(joints, dict):
            simplified_joints = dict()
            for joint in joints:
                # Pull out the name of the joint
                joint_name = '_'.join(joint.split('_')[1::])
                simplified_joints[joint] = simplify_angle(joints[joint])
        elif isinstance(joints, list):
            simplified_joints = []
            #separate the joint name from the group name
            joint_order = map(lambda s: "_".join(s.split("_")[1::]), 
                              ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4",
                        "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"])
            
            continuous_joint_indices = [joint_order.index(j) for j in ["joint_1", "joint_2", "joint_3", "joint_4",
                        "joint_5", "joint_6", "joint_7"]]

            for i in range(len(joints)):
                a = joints[i]
                if i in continuous_joint_indices:
                    simplified_joints.append(simplify_angle(a))
                else:
                    simplified_joints.append(a)
        else:
            rospy.logerr("Joints must be provided as a list or dictionary")
            raise TypeError("Joints must be provided as a list or dictionary")
        return simplified_joints


if __name__ == '__main__':

    #args = argumentParser(None)

    kinova_robotTypeParser("j2s7s300")
    rospy.init_node(prefix + 'pose_action_client')

    """if args.unit == 'mq':
        if len(args.pose_value) != 7:
            print('Number of input values {} is not equal to 7 (3 position + 4 Quaternion).'.format(len(args.pose_value)))
            sys.exit(0)
    elif (args.unit == 'mrad') | (args.unit == 'mdeg'):
        if len(args.pose_value) != 6:
            print('Number of input values {} is not equal to 6(3 position + 3 EulerAngles).'.format(len(args.pose_value)))
            sys.exit(0)
    else:
        raise Exception('Cartesian value have to be in unit: mq, mdeg or mrad')"""

    getcurrentCartesianCommand(prefix)

    """for i in range(300):
        pose = [-0.01, 0.001, 0.0, 0.0, 0.0, 0.0]

        pose_mq, pose_mdeg, pose_mrad = unitParser('mdeg', pose, True)
        
        try:

            poses = [float(n) for n in pose_mq]

            result = cartesian_pose_client(poses[:3], poses[3:])

            print('Cartesian pose sent!')

        except rospy.ROSInterruptException:
            print "program interrupted before completion"


        verboseParser(True, poses)"""