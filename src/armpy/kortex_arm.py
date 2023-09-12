#!/usr/bin/env python3

"""
File for doing manipulation stuff with the arm.
Largely uses code from ros_kortex

Author: Isaac Sheidlower, AABL Lab, Isaac.Sheidlower@tufts.edu
"""
from math import radians
import message_filters
import threading
from turtle import pos
import numpy as np
import sys
import rospy
import time
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import RobotState, PositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kortex_driver.srv import *
from kortex_driver.msg import *

try:
    import aiorospy
except ImportError:
    HAS_AIOROSPY = False
else:
    HAS_AIOROSPY = True
    import asyncio


class Arm:
    def __init__(self):
        try:
            try:
                rospy.init_node('arm_movement')
            except:
                pass
            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            param_list = rospy.get_param_names()
            #print(param_list)
            if '/my_gen3_lite/robot_description' in param_list:
                # Get node params
                self.robot_name = rospy.get_param('~robot_name', default="my_gen3_lite")
                #print(self.robot_name)
                self.degrees_of_freedom = rospy.get_param(
                    "/" + self.robot_name + "/degrees_of_freedom", default=6)
                #print("DOF IS: " + str(self.degrees_of_freedom))
                self.is_gripper_present = rospy.get_param(
                    "/" + self.robot_name + "/is_gripper_present", False)
            else:
                # Get node params
                self.robot_name = rospy.get_param('~robot_name', default="my_gen3")
                #print(self.robot_name)
                self.degrees_of_freedom = rospy.get_param(
                    "/" + self.robot_name + "/degrees_of_freedom", default=7)
                #print("DOF IS: " + str(self.degrees_of_freedom))
                self.is_gripper_present = rospy.get_param(
                    "/" + self.robot_name + "/is_gripper_present", False)
                
            

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) +
                          " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # setup defult translation and orientation speed
            # when moving in cartesian space
            self.cartesian_speed = CartesianSpeed()
            self.cartesian_speed.translation = .1  # m/s
            self.cartesian_speed.orientation = 15  # deg/s

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber(
                "/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(
                clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(
                read_action_full_name, ReadAction)

            self.stop = rospy.ServiceProxy(
                f"/{self.robot_name}/base/stop", Stop)
            self.estop = rospy.ServiceProxy(
                f"{self.robot_name}/base/apply_emergency_stop", ApplyEmergencyStop)


            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(
                execute_action_full_name, ExecuteAction)
          
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + \
                '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(
                set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + \
                self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(
                activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(
                activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            send_gripper_command_full_name = '/' + \
                self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(
                send_gripper_command_full_name, SendGripperCommand)

            get_product_configuration_full_name = '/' + \
                self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(
                get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + \
                self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(
                validate_waypoint_list_full_name, ValidateWaypointList)
            
            rospy.wait_for_service(f"/{self.robot_name}/base/play_joint_trajectory")
            rospy.wait_for_service(f"/{self.robot_name}/base/play_cartesian_trajectory")
            
            self.clear_faults()
            self.set_cartesian_reference_frame()
            self.subscribe_to_a_robot_notification()

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        """
        Note: not sure what blending_radius is but defaulting to zero everywhere this function is called
        """
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(
            cartesianWaypoint)

        return waypoint

    async def action_complete(self, message_timeout=0.5):
        """
        Coroutine to block until an action is complete.

        message_timeout: Duration to wait for a notification on the action_topic topic

        Raises:
            TimeoutError: No message received or rospy shutdown
            CancelledError: Action aborted or rospy shutdown
        """

        if not HAS_AIOROSPY:
            raise NotImplementedError()

        action_event_sub = aiorospy.AsyncSubscriber(f"{self.robot_name}/action_topic", ActionNotification, 
                                                   queue_size=1) # always get newest message
        sub_it = action_event_sub.subscribe().__aiter__()
        try:
            while not rospy.is_shutdown():
                evt = await asyncio.wait_for(sub_iter.__anext__(), message_timeout)
                if evt.action_event == ActionEvent.ACTION_END:
                    return True
                elif evt.action_event == ActionEvent.ACTION_ABORT:
                    raise asyncio.CancelledError()
        # shut down the generator -- therefore the subscriber
        finally:
            await sub_it.aclose()
            
        # rospy shutdown
        raise asyncio.CancelledError()


    def wait_for_action_end_or_abort(self):
        """
        Function to wait for an action to end or abort based on
        kinovas action topics. If this is not called, then there is
        a chance that proceeding action requests will not be executed.
        """
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                rospy.sleep(0.01)

    def clear_faults(self, block=True):
        """
        Clears the robots faults. I belive this means clearing any prior
        collisions so the robot no longer thinks it is in collision.
        """
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            if block:
                rospy.sleep(2.5)
            return True

    def subscribe_to_a_robot_notification(self, block=True):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")
        if block:
            rospy.sleep(1.0)

        return True

    def home_arm(self, block=True):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                if block:
                    return self.wait_for_action_end_or_abort()
                else:
                    return True

    def get_ik(self, pose=None, check_collisions=True):
        """
        Returns list of joints for a given pose. If no pose is give,
        will return ik for current joints. If no IK solution is found
        will return -1. If the pose is in collision, will return -2. 

        pose: list, numpy array, or PoseStamped message
        If list or numpy array, first three positions should be x,y,z position
        and next for positions should be x,y,z,w in quaternian. 
        """

        rospy.wait_for_service(f"/{self.robot_name}/compute_ik")
        compute_ik = rospy.ServiceProxy(f"/{self.robot_name}/compute_ik", GetPositionIK)
        if pose is None:
            data = rospy.wait_for_message(
                f"/{self.robot_name}/base_feedback", BaseCyclic_Feedback)
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = data.base.tool_pose_x
            pose_stamped.pose.position.z = data.base.tool_pose_z
            pose_stamped.pose.position.y = data.base.tool_pose_y

            quat = quaternion_from_euler(
                data.base.tool_pose_theta_x, data.base.tool_pose_theta_y, data.base.tool_pose_theta_z)
            # print(quat)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]
            # print(pose_stamped)
        else:
            if isinstance(pose, (list, np.ndarray)):
                temp_pose = PoseStamped()
                temp_pose.pose.position.x = pose[0]
                temp_pose.pose.position.y = pose[1]
                temp_pose.pose.position.z = pose[2]
                temp_pose.pose.orientation.x = pose[3]
                temp_pose.pose.orientation.y = pose[4]
                temp_pose.pose.orientation.z = pose[5]
                temp_pose.pose.orientation.w = pose[6]
                pose_stamped = temp_pose
            else:
                pose_stamped = pose

        # print(pose_stamped)
        #pose.position.x = data.base.tool_pose_x
        #pose.position.x = data.base.tool_pose_x
        #pose.position.x = data.base.tool_pose_x

        # with moveit
        ik_req = PositionIKRequest()
        ik_req.group_name = "arm"
        ik_req.pose_stamped = pose_stamped
        ik_req.robot_state.is_diff = True
        if check_collisions:
            ik_req.avoid_collisions = True
        else:
            ik_req.avoid_collisions = False

        ik_result = compute_ik(ik_req)
        if ik_result.error_code.val == -31:
            return -1
        elif ik_result.error_code.val == -12:
            return -2
        else:
            if self.degrees_of_freedom == 7:
                return ik_result.solution.joint_state.position[0:7]
            else:
                return ik_result.solution.joint_state.position[0:6]
        return ik_result

    def get_fk(self, joints=None):
        """
        Returns fk of joints as a PoseStamped message. If no 
        fk solution is found will return -1.

        joints: list or dictionary of joints, 
            lists are assumed to be in order of joint1-6/7
        """

        if joints is None:
            joint_state = rospy.wait_for_message(
                f"/{self.robot_name}/joint_states", JointState)
        else:
            joint_message = JointState()
            if isinstance(joints, list):
                if self.degrees_of_freedom == 7:
                    joint_message.name = [
                        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
                else:
                    joint_message.name = [
                        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
                joint_message.position = joints
            else:
                joint_message.name = list(joints.keys())
                joint_message.position = joints.values()

            joint_state = joint_message

        compute_fk = rospy.ServiceProxy(f"/{self.robot_name}/compute_fk", GetPositionFK)

        robot_state = RobotState()
        robot_state.joint_state = joint_state

        fk_request = GetPositionFK()

        fk_request.robot_state = robot_state
        result = compute_fk.call(
            fk_link_names=['tool_frame'], robot_state=robot_state)
        if not result.error_code.val == 1:
            return -1
        else:
            return result.pose_stamped[0]

    def get_eef_pose(self, quaternion=True):
        """
        Returns current eef pose as a PoseStamped if quaternion is True,
        otherwise returns a list of [x,y,z,theta_x,theta_y,theta_z] in radians
        """
        data = rospy.wait_for_message(
            f"/{self.robot_name}/base_feedback", BaseCyclic_Feedback)
        if quaternion:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = data.base.tool_pose_x
            pose_stamped.pose.position.z = data.base.tool_pose_z
            pose_stamped.pose.position.y = data.base.tool_pose_y
            
            quat = quaternion_from_euler(
                data.base.tool_pose_theta_x, data.base.tool_pose_theta_y, data.base.tool_pose_theta_z)
            # print(quat)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            return pose_stamped
        else:
            theta_x = np.deg2rad(data.base.tool_pose_theta_x)
            theta_y = np.deg2rad(data.base.tool_pose_theta_y)
            theta_z = np.deg2rad(data.base.tool_pose_theta_z)
            return [data.base.tool_pose_x, data.base.tool_pose_y, data.base.tool_pose_z, theta_x, theta_y, theta_z]
            
    def get_joint_angles(self):
        """
        Returns current joints as a list in order of joints
        """
        joint_states = rospy.wait_for_message(
            f"/{self.robot_name}/joint_states", JointState)

        if self.degrees_of_freedom == 7:
            return joint_states.position[0:7]
        else:
            return joint_states.position[0:6]

    def set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def goto_cartesian_pose(self, pose, relative=False, check_collision=False, wait_for_end=True,
                            translation_speed=None, orientation_speed=None, radians=False):
        """
        Function that goes to a cartesian pose using ros_kortex's provided interface.
        pose: list, numpy array, or PoseStamped message
            If list or numpy array, first three positions should be x,y,z position
            and next for positions should be x,y,z,w in quaternian. 

        relative: If relative is False, the arm will go to the cartesian pose specified
        by the "pose" argument. Else if relative is true, then the the arms current cartesian 
        pose will be incremented with the passed "pose" argument. Else

        check_collision: If check_collision=True, the function will check if 
        the robot is in collision and return -1. If it is not in collision it will return 1.

        wait_for_end: If wait_for_end=True, the function will return only after the action
        is completed or the action aborts.

        if translation_speed(m/s) or orientation_speed(deg/s) is not None, the default will be used.

        radians: If radians=True, the orientation will be specified in radians (expects array
        of with 6 values).

        TODO: fill out the functionality of the remaining arguments
        """
        self.subscribe_to_a_robot_notification()
        self.clear_faults()
        if isinstance(pose, (list, np.ndarray)):
            if radians is False:
                temp_pose = PoseStamped()
                temp_pose.pose.position.x = pose[0]
                temp_pose.pose.position.y = pose[1]
                temp_pose.pose.position.z = pose[2]
                temp_pose.pose.orientation.x = pose[3]
                temp_pose.pose.orientation.y = pose[4]
                temp_pose.pose.orientation.z = pose[5]
                temp_pose.pose.orientation.w = pose[6]
                pose = temp_pose

                euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                    pose.pose.orientation.z, pose.pose.orientation.w))
            else:
                temp_pose = PoseStamped()
                temp_pose.pose.position.x = pose[0]
                temp_pose.pose.position.y = pose[1]
                temp_pose.pose.position.z = pose[2]
                # dummy valuesL
                temp_pose.pose.orientation.x = 0
                temp_pose.pose.orientation.y = 0
                temp_pose.pose.orientation.z = 0
                temp_pose.pose.orientation.w = 0

                euler_corr = [pose[3], pose[4], pose[5]]
                pose = temp_pose

        else:
            euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                pose.pose.orientation.z, pose.pose.orientation.w))

        euler_corr = np.rad2deg(euler_corr)
        cartesian_speed = CartesianSpeed()
        if translation_speed is not None:
            cartesian_speed.translation = translation_speed
        else:
            cartesian_speed.translation = self.cartesian_speed.translation

        if orientation_speed is not None:
            cartesian_speed.orientation = orientation_speed
        else:
            cartesian_speed.orientation = self.cartesian_speed.orientation

        # set-up goal:
        #euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
        #                                    pose.pose.orientation.z, pose.pose.orientation.w))

        constrained_pose = ConstrainedPose()
        constrained_pose.constraint.oneof_type.speed.append(cartesian_speed)

        if relative is False:
            constrained_pose.target_pose.x = pose.pose.position.x
            constrained_pose.target_pose.y = pose.pose.position.y
            constrained_pose.target_pose.z = pose.pose.position.z
            constrained_pose.target_pose.theta_x = euler_corr[0]
            constrained_pose.target_pose.theta_y = euler_corr[1]
            constrained_pose.target_pose.theta_z = euler_corr[2]
        else:
            feedback = rospy.wait_for_message(
                "/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            constrained_pose.target_pose.x = feedback.base.commanded_tool_pose_x+pose.pose.position.x
            constrained_pose.target_pose.y = feedback.base.commanded_tool_pose_y+pose.pose.position.y
            constrained_pose.target_pose.z = feedback.base.commanded_tool_pose_z+pose.pose.position.z
            constrained_pose.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x + \
                euler_corr[0]
            constrained_pose.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y + \
                euler_corr[1]
            constrained_pose.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z + \
                euler_corr[2]

        # print(constrained_pose.target_pose)
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(constrained_pose)
        req.input.name = "pose"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose")
            success = False
        else:
            rospy.loginfo("Waiting for pose to finish...")

        #print("arm")
        # self.wait_for_action_end_or_abort()
        #print("done")

        return 1

    def goto_eef_pose(self, pose):
        """
        Function that goes to a cartesian pose using ros_kortex's provided interface.
        pose: list, numpy array, or PoseStamped message
            If list or numpy array, first three positions should be x,y,z position
            and next for positions should be x,y,z,w in quaternian. 
        TODO: add functionality for different eef frames
        """
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message(
            "/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        if isinstance(pose, (list, np.ndarray)):
            temp_pose = PoseStamped()
            temp_pose.pose.position.x = pose[0]
            temp_pose.pose.position.y = pose[1]
            temp_pose.pose.position.z = pose[2]
            temp_pose.pose.orientation.x = pose[3]
            temp_pose.pose.orientation.y = pose[4]
            temp_pose.pose.orientation.z = pose[5]
            temp_pose.pose.orientation.w = pose[6]
            pose = temp_pose

            euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                pose.pose.orientation.z, pose.pose.orientation.w))
        else:
            euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                pose.pose.orientation.z, pose.pose.orientation.w))
        # Possible to execute waypointList via execute_action service or use execute_waypoint_trajectory service directly
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                euler_corr[0],
                euler_corr[1],
                euler_corr[2],
                0
            )
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Call the service
        rospy.loginfo("Sending the robot to the eef pose...")
        try:
            self.execute_action(req)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        # else:
            # TODO: This function seems a bit finiky, investigate.
            # return self.wait_for_action_end_or_abort()

    def goto_joint_pose(self, joints, radians=True, block=True):
        """
        Sends the arm to the specified joint angles. 
        joints: list of joint anlges (from 1 to 7 or from 1 to 6 for lite)
        radians: If true, angles will be considered in radians, 
            default is degrees.
        TODO: add dictionary functionality
        """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        if radians:
            for angle in range(self.degrees_of_freedom):
                angularWaypoint.angles.append(np.degrees(joints[angle]) % 360)
        else:
            for angle in range(self.degrees_of_freedom):
                angularWaypoint.angles.append(joints[angle])

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(
            angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(
            res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION):
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(
                res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION):
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Send the angles
        rospy.loginfo("Moving to joint pose")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            if block:
                return self.wait_for_action_end_or_abort()
            else:
                return True

    def goto_zero(self, block=True):
        """
            Sends the arm fully vertical where all the joints are zero.
        """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm to vertical position (all zeros)
        for _ in range(self.degrees_of_freedom):
            angularWaypoint.angles.append(0.0)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(
            angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(
            res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION):
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(
                res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION):
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            if block:
                return self.wait_for_action_end_or_abort()
            else:
                return True

    def goto_eef_waypoints(self, waypoints):
        """
            Send the arm through a list of waypoints. 
            Each waypoint may be list, numpy array, or a list of PoseStamped messages
            TODO: add functionality for different eef frames
        """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        for pose in waypoints:
            if isinstance(pose, (list, np.ndarray)):
                temp_pose = PoseStamped()
                temp_pose.pose.position.x = pose[0]
                temp_pose.pose.position.y = pose[1]
                temp_pose.pose.position.z = pose[2]
                temp_pose.pose.orientation.x = pose[3]
                temp_pose.pose.orientation.y = pose[4]
                temp_pose.pose.orientation.z = pose[5]
                temp_pose.pose.orientation.w = pose[6]
                pose = temp_pose

                euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                    pose.pose.orientation.z, pose.pose.orientation.w))
            else:
                euler_corr = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y,
                                                    pose.pose.orientation.z, pose.pose.orientation.w))

            trajectory.waypoints.append(self.FillCartesianWaypoint(pose.pose.position.x,  pose.pose.position.y,
                                                                   pose.pose.position.z,  euler_corr[0], euler_corr[1], euler_corr[2], 0))

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Call the service
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        # else:
            # TODO: This function seems a bit finiky, investigate.
            # return self.wait_for_action_end_or_abort()

    def goto_joint_waypoints(self, waypoints, radians=False, block=True):
        """
        NOTE: Currently this is not functional, not sure why it does not work. 

        Sends the arm to the specified series joint angles. 
        joints: list of joint anlges (from 1 to 7)
        radians: If true, angles will be considered in radians, 
            default is degrees.
        TODO: add dictionary functionality
        """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        for i, joints in enumerate(waypoints):
            waypoint = Waypoint()
            angularWaypoint = AngularWaypoint()

            if radians:
                for angle in range(self.degrees_of_freedom):
                    angularWaypoint.angles.append(
                        np.degrees(joints[angle]) % 360)
            else:
                for angle in range(self.degrees_of_freedom):
                    angularWaypoint.angles.append(joints[angle])

            # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
            # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
            angular_duration = 0
            angularWaypoint.duration = angular_duration

            # Initialize Waypoint and WaypointList
            waypoint.oneof_type_of_waypoint.angular_waypoint.append(
                angularWaypoint)

            trajectory.waypoints.append(waypoint)
        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(
            res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION):
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(
                res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION):
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Send the angles
        rospy.loginfo("Moving to joint pose")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            if block:
                return self.wait_for_action_end_or_abort()
            else:
                return True

    def close_gripper(self, **kwargs):
        """
        Fully closes the gripper
        """
        return self.gripper_command(1., relative=False, mode="position", **kwargs)

    def open_gripper(self, **kwargs):
        """
        Fully opens the gripper.
        """
        return self.gripper_command(0., relative=False, mode="position", **kwargs)

    def gripper_command(self, value, relative=False, mode="position", duration=0, block=True):
        """
        Closes/opens the griper to the specified value. 
        The value is between 0 (fully open) and 1 (fully closed)

        duration in milliseconds, only used in speed mode
        """
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        if relative is False:
            finger.value = value
        else:
            finger.value = value + self.get_gripper_position()
        req.input.gripper.finger.append(finger)
        if mode == "position":
            req.input.mode = GripperMode.GRIPPER_POSITION
        elif mode == "speed":
            req.input.mode = GripperMode.GRIPPER_SPEED
            req.input.duration = duration
        elif mode == "force":
            req.input.mode = GripperMode.GRIPPER_FORCE

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
            if block:
                rospy.sleep(0.5)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        finally:
            if block:
                rospy.sleep(0.5)
            return True
    
    def get_gripper_position(self):
        """
        Returns the position of the gripper in the range 0 (open) to 1 (closed)
        """
        if self.degrees_of_freedom == 7:
            return rospy.wait_for_message(f"/{self.robot_name}/base_feedback/joint_state", JointState).position[7]
        else:
            return rospy.wait_for_message(f"/{self.robot_name}/base_feedback/joint_state", JointState).position[6]
        
    async def gripper_command_async(self, value, *args, message_timeout=1, tolerance=1e-3, **kwargs):
        """
        asyncio coroutine that sends a gripper command and then waits for the gripper value to match
        recommended to wrap this in an asyncio.wait_for

        value: gripper value to send
        *args: forwarded to gripper_command
        message_timeout: duration (sec) to wait for a joint state message
        tolerance: value tolerance to say location has been reached
        **kwargs: forwarded to gripper_command

        raises:
            TimeoutError: if a JointState message is not received within message_timeout
            CancelledError: if rospy is shut down before goal is reached
                Note that due to timeout effects, it is indeterminate whether TimeoutError or
                CancelledError will be called if rospy is shutdown
        """
        if not HAS_AIOROSPY:
            raise NotImplementedError()
        
        kwargs["block"] = False
        self.gripper_command(value, *args, **kwargs)

        gripper_pos_sub = aiorospy.AsyncSubscriber(f"{self.robot_name}/base_feedback/joint_state", JointState, 
                                                   queue_size=1) # always get newest message
        sub_it = gripper_pos_sub.subscribe().__aiter__()
        try:
            while not rospy.is_shutdown():
                msg = await asyncio.wait_for(sub_it.__anext__(), message_timeout)
                # todo: check that velocity is also ~0?
                if np.abs(msg.position[self.degrees_of_freedom] - value) < tolerance:
                    return True
        # shut down the generator -- therefore the subscriber
        finally:
            await sub_it.aclose()

        # rospy shutdown
        raise asyncio.CancelledError()

    def joint_velocity_command(self, values, duration, duration_timeout=None, collision_check=False):
        """
        Sends velocity commads to the joints for the specified duration. 
        Returns a 1 on completion. 

        --------------
        values: A list or numpy array of joint velocity. 
        If it is a list or dictionary, it is assumed to be ordered 1-7.

        duration: The length of time the robot will execute the velocity command for.

        duration_timeout: if None, the function will return after duration. Else,
        the function will return after duration timeout even though the arm
        will move for time=duration. Currently this functionionallity uses
        python threading so use with caution. 

        collision_check: if True, will calculate the arms furture position and return -1 if
        it is in collision.

        TODO: -add dictionary functionality
              -add collision check
        """

        def velocity_command(values, duration):
            joint_vel_publisher = rospy.Publisher(
                f"/{self.robot_name}/in/joint_velocity", Base_JointSpeeds, queue_size=10, latch=True)
            joint_speeds = Base_JointSpeeds()
            joint_speeds.duration = 0
            for i in range(len(values)):
                speed = JointSpeed()
                speed.joint_identifier = i
                speed.value = values[i]
                joint_speeds.joint_speeds.append(speed)

            joint_vel_publisher.publish(joint_speeds)
            rospy.sleep(duration)
            self.stop()
            return 1

        if duration_timeout is not None:
            move_thread = threading.Thread(
                target=velocity_command, args=(values, duration))
            move_thread.start()
            rospy.sleep(duration_timeout)
            return 1
        else:
            return velocity_command(values, duration)

    def cartesian_velocity_command(self, values, duration, duration_timeout=None, collision_check=False, radians=False, block=True):
        """
        Sends a carteian velocity command for a specified duration. 
        Returns 1 on completion.

        ---------
        values: list or np array of the form: [x,y,z, x-twist, y-twist, z-twist, w-twist]

        duration: The length of time the robot will execute the velocity command for.

        duration_timeout: if None, the function will return after duration. Else,
        the function will return after duration timeout even though the arm
        will move for time=duration. Currently this functionionallity uses
        python threading so use with caution. 

        collision_check: if True, will calculate the arms furture position and return -1 if
        it is in collision.

        block: if True, follows duration_timeout; otherwise returns immediately

        TODO: add collision check
        """

        def velocity_command(values, duration):
            cartesian_vel_publisher = rospy.Publisher(
                f"/{self.robot_name}/in/cartesian_velocity", TwistCommand, queue_size=1, latch=True)
           
            empty_message = std_msgs.msg.Empty()
            cartesian_command = TwistCommand()

            # NOTE: There is a bug in the mapping of values/twists to angles
            # here that needs to be figured out
            if not radians:
                twists = euler_from_quaternion((values[3:]))
                cartesian_command.twist.linear_x = values[0]
                cartesian_command.twist.linear_y = values[1]
                cartesian_command.twist.linear_z = values[2]
                cartesian_command.twist.angular_x = twists[0]
                cartesian_command.twist.angular_y = twists[1]
                cartesian_command.twist.angular_z = twists[2]
            else:
                cartesian_command.twist.linear_x = values[0]
                cartesian_command.twist.linear_y = values[1]
                cartesian_command.twist.linear_z = values[2]
                cartesian_command.twist.angular_x = values[3]
                cartesian_command.twist.angular_y = values[4]
                cartesian_command.twist.angular_z = values[5]

            cartesian_command.reference_frame = 0
            # print(cartesian_command)
            cartesian_vel_publisher.publish(cartesian_command)
            if block:
                rospy.sleep(duration)
                #stop_publisher.publish(empty_message)
                # this sleep is necessary to process the sleep before the next potential command
                rospy.sleep(.00000001)
            return 1

        if duration_timeout is not None and block:
            move_thread = threading.Thread(
                target=velocity_command, args=(values, duration))
            move_thread.start()
            rospy.sleep(duration_timeout)
            return 1
        else:
            return velocity_command(values, duration)

    def stop_arm(self):
        """
        Stops the arm from moving
        """
        self.stop()
        return
    
    def goto_joint_pose_sim(self, joints):
        """              
        Sends the arm to the specified joint angles (in radians).
        This fucntion returns before completeley getting to 
        the pose

        Note: this function is primarily for the gazebo sim of the arm
        and may not work with the real arm. With the real arm 
        goto_joint_pose can be used.

        TODO: Figure out the difference/what is better between this
        and goto_joint_pose

        --------
        joints: list or np array of joint anlges (from 1 to 7) 
        """

        go_to_joints = rospy.ServiceProxy(
            f"/{self.robot_name}/base/play_joint_trajectory", PlayJointTrajectory)

        joint_angle_arr = ConstrainedJointAngles()

        for i in range(self.degrees_of_freedom):
            temp_joint = JointAngle(
                joint_identifier=i, value=np.rad2deg(joints[i]))
            joint_angle_arr.joint_angles.joint_angles.append(temp_joint)

        joint_msg = PlayJointTrajectoryRequest()
        joint_msg.input = joint_angle_arr
        go_to_joints(joint_msg)
        return
    
    def goto_joint_pose_relative_sim(self, joints):
        """              
        Sends the arm to the relative specified joint angles (in radians).
        This fucntion returns before completeley getting to 
        the pose

        Note: this function is primarily for the gazebo sim of the arm
        and may not work with the real arm. With the real arm 
        goto_joint_pose can be used.
        --------------
        joints: list or np array of joint anlges (from 1 to 7) 
        """
    
        go_to_joints = rospy.ServiceProxy(
            f"/{self.robot_name}/base/play_joint_trajectory", PlayJointTrajectory)
        
        current_joints = self.get_joint_angles()

        joint_angle_arr = ConstrainedJointAngles()

        for i in range(self.degrees_of_freedom):
            temp_joint = JointAngle(
                joint_identifier=i, value=np.rad2deg(current_joints[i] + joints[i]))
            joint_angle_arr.joint_angles.joint_angles.append(temp_joint)
        
        joint_msg = PlayJointTrajectoryRequest()
        joint_msg.input = joint_angle_arr
        go_to_joints(joint_msg)
        return


    def goto_cartesian_pose_sim(self, pose, speed=.1, duration=None, radians=True):
        """
        Sends the arm to the specified cartesian pose within a specified duration. 
        This function returns before completely getting to the pose.

        NOTE: there is a specific relationship between the duration and speed.
        If the duration is set to a number, it overrides the speed. So a 
        duration of 1 second will take 1 second regardless of the speed (if
        the position is reachable).
        NOTE: this function is primarily for the gazebo sim of the arm
        and does not with the real arm. With the real arm goto_cartesian_pose
        can be used.

        --------
        pose: list or np array of the form [x,y,z, x-rot, y-rot, z-rot], rotations in Euler
        radians: if True, the rotation values are in radians. Else, they are in degrees
        speed: the speed of the arm in m/s. The speed is for translation and orientation
        duration (int): the duration alloted for the arm to reach the goal pose. Default is None
        to prioritize speed over duration.

        TODO: add quaternion support
        TODO: maybe add speed for translation and orientation separately
        """

        go_to_cart = rospy.ServiceProxy(
            f"/{self.robot_name}/base/play_cartesian_trajectory", PlayCartesianTrajectory)
        cart_pose = ConstrainedPose()
        if radians:
            cart_pose.target_pose.x = pose[0]
            cart_pose.target_pose.y = pose[1]
            cart_pose.target_pose.z = pose[2]
            cart_pose.target_pose.theta_x = np.rad2deg(pose[3])
            cart_pose.target_pose.theta_y = np.rad2deg(pose[4])
            cart_pose.target_pose.theta_z = np.rad2deg(pose[5])
        else:
            cart_pose.target_pose.x = pose[0]
            cart_pose.target_pose.y = pose[1]
            cart_pose.target_pose.z = pose[2]
            cart_pose.target_pose.theta_x = pose[3]
            cart_pose.target_pose.theta_y = pose[4]
            cart_pose.target_pose.theta_z = pose[5]

        constraint = CartesianTrajectoryConstraint_type()
        if duration is not None:
            constraint.duration = [duration]
        constraint.speed = [CartesianSpeed(speed, speed)]

        cart_msg = PlayCartesianTrajectoryRequest()
        cart_msg.input.target_pose = cart_pose
        cart_pose.constraint.oneof_type = constraint

        # not sure if this constraint line changes anything
        cart_msg.input.constraint = constraint
        go_to_cart(cart_pose)
        return

    def goto_cartesian_relative_sim(self, pose, speed=.1, duration=None, radians=True):
        """
        Sends the arm via a displaced pose command relative to its current pose.

        NOTE: there is a specific relationship between the duration and speed.
        If the duration is set to a number, it overrides the speed. So a 
        duration of 1 second will take 1 second regardless of the speed (if
        the position is reachable).
        NOTE: this function is primarily for the gazebo sim of the arm
        though it can work with the real arm. With the goto_cartesian_pose
        can be used.
            try:
                rospy.init_node('arm_movement')
            except:
                pass

        --------
        pose: list or np array of the form [x,y,z, x-rot, y-rot, z-rot], rotations in Euler
        radians: if True, the rotation values are in radians. Else, they are in degrees
        speed: the speed of the arm in m/s. The speed is for translation and orientation
        duration (int): the duration alloted for the arm to reach the goal pose. Default is None
        to prioritize speed over duration.

        TODO: add quaternion support
        TODO: maybe add speed for translation and orientation separately
        """

        go_to_cart = rospy.ServiceProxy(
            f"/{self.robot_name}/base/play_cartesian_trajectory", PlayCartesianTrajectory)
        cart_pose = ConstrainedPose()

        curr_pose = self.get_eef_pose(quaternion=False)
        if radians:
            cart_pose.target_pose.x = curr_pose[0]+pose[0]
            cart_pose.target_pose.y = curr_pose[1]+pose[1]
            cart_pose.target_pose.z = curr_pose[2]+pose[2]
            cart_pose.target_pose.theta_x = np.rad2deg(curr_pose[3]+pose[3])
            cart_pose.target_pose.theta_y = np.rad2deg(curr_pose[4]+pose[4])
            cart_pose.target_pose.theta_z = np.rad2deg(curr_pose[5]+pose[5])
        else:
            cart_pose.target_pose.x = pose[0]
            cart_pose.target_pose.y = pose[1]
            cart_pose.target_pose.z = pose[2]
            cart_pose.target_pose.theta_x = np.rad2deg(curr_pose[3])+pose[3]
            cart_pose.target_pose.theta_y = np.rad2deg(curr_pose[4])+pose[4]
            cart_pose.target_pose.theta_z = np.rad2deg(curr_pose[5])+pose[5]

        constraint = CartesianTrajectoryConstraint_type()
        if duration is not None:
            constraint.duration = [duration]
        constraint.speed = [CartesianSpeed(speed, speed)]

        cart_msg = PlayCartesianTrajectoryRequest()
        cart_msg.input.target_pose = cart_pose
        cart_pose.constraint.oneof_type = constraint

        # not sure if this constraint line changes anything
        cart_msg.input.constraint = constraint
        go_to_cart(cart_pose)
        return
    
    def get_feedback_sub_args(self):
        return f'{self.robot_name}/base_feedback', BaseCyclic_Feedback
    
