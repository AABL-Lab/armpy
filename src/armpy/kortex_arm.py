#!/usr/bin/env python3

"""
File for doing manipulation stuff with the arm.
Largely uses code from ros_kortex

Author: Isaac Sheidlower, AABL Lab, Isaac.Sheidlower@tufts.edu
"""
import copy
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
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf.transformations as tfs
from kortex_driver.srv import *
from kortex_driver.msg import *

try:
    import aiorospy
except ImportError:
    HAS_AIOROSPY = False
else:
    HAS_AIOROSPY = True
    import asyncio


#############
## PARSING/CONVERTING MESSAGES
##############
def pose_to_kortex_pose(pose):
    # cheap and dirty way to accept PoseStamped
    if hasattr(pose, "pose"):
        pose = pose.pose

    rpy = np.rad2deg(euler_from_quaternion((
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )))

    return Pose(
        x = pose.position.x,
        y = pose.position.y,
        z = pose.position.z,
        theta_x = rpy[0],
        theta_y = rpy[1],
        theta_z = rpy[2]
    )

def pose_tf_to_kortex_pose(pose):
    p = tfs.translation_from_matrix(pose)
    r = np.rad2deg(tfs.euler_from_matrix(pose))
    return Pose(*p, *r)

def pose_pq_to_kortex_pose(p, q):
    return Pose(*p, *np.rad2deg(tfs.euler_from_quaternion(q)))

def kortex_pose_to_position_euler(pose, prefix=""):
    return (
        (getattr(pose, prefix+"x"),
            getattr(pose, prefix+"y"),
            getattr(pose, prefix+"z")),
        np.deg2rad((
            getattr(pose, prefix+"theta_x"), 
            getattr(pose, prefix+"theta_y"),
            getattr(pose, prefix+"theta_z"))
        )
    )

def kortex_pose_to_pose(pose, prefix):
    p, r = kortex_pose_to_position_euler(pose, prefix)
    return geometry_msgs.msg.Pose(
        position = geometry_msgs.msg.Point(*p),
        orientation = geometry_msgs.msg.Quaternion(*tfs.quaternion_from_euler(*r))
    )

def kortex_pose_to_transformation_matrix(pose, prefix):
    p, r = kortex_pose_to_position_euler(pose, prefix)
    return tfs.concatenate(
        tfs.translation_matrix(p),
        tfs.euler_matrix(*r)
    )

def parse_pose_input(pose):
    ## parse pose input
    if isinstance(pose, (geometry_msgs.msg.Pose, geometry_msgs.msg.PoseStamped)):
        if isinstance(pose, geometry_msgs.msg.PoseStamped):
            pose = pose.pose
        p = (pose.position.x, pose.position.y, pose.position.z)
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    elif isinstance(pose, (list, np.ndarray)):
        p = pose[:3]
        if len(pose[3:]) == 0:
            q = [0, 0, 0, 1]
        elif len(pose[3:]) == 3:
            # euler angles, always radians
            q = quaternion_from_euler(pose[3:])
        elif len(pose[3:]) == 4:
            # TODO: validate
            q = pose[3:]
        else:
            raise ValueError(f"values as pose must be length 3, 6, or 7, got {pose}")
    else:
        raise ValueError(f"Unknown pose input: {pose}")
    return p, q

# TODO: figure out how to do this and deal with frame_id
# try:
#     import tf2_ros
# except ImportError:
#     pass
# else:
#     tf2_ros.ConvertRegistration().add_


# set up services for lazy definition/acquisition
# is this too fancy? idk seems not ideal to set up all dozens of services if unused
_KORTEX_SERVICES = {
    "compute_fk": ("/compute_fk", GetPositionFK),
    "compute_ik": ("/compute_ik", GetPositionIK),
    "get_product_configuration": ("/base/get_product_configuration", GetProductConfiguration),
    "read_sequence": ("/base/read_sequence", ReadSequence),
    "read_all_sequences": ("/base/read_all_sequences", ReadAllSequences),
    "play_sequence": ("/base/play_sequence", PlaySequence),
    "play_joint_trajectory" : ("/my_gen3_lite/base/play_joint_trajectory", PlayJointTrajectory),
    "play_precomputed_joint_trajectory": ("/base/play_precomputed_trajectory", PlayPreComputedJointTrajectory),
}

_DEFAULT_DOF = {
    "gen3": 7,
    "gen3_lite": 6
}

class Arm:
    HOME_ACTION_IDENTIFIER = 2

    def __init__(self, robot_name=None):
        rospy.loginfo(f"Loading robot {robot_name}")

        # figure out which robot we are, gen3 or gen3_lite
        # step 1: use arm_name
        if robot_name is not None:
            # easy?
            self.robot_name = robot_name
            self.degrees_of_freedom = _DEFAULT_DOF.get(robot_name)
        else:
            # no arm_name provided
            # next place to check is the parameter server
            # TODO: do we want search_param instead?
            self.robot_name = rospy.get_param('~robot_name', default=None)

            if self.robot_name is None:
                # no dice, let's make some guesses

                if rospy.has_param('/my_gen3_lite/robot_description'):
                    # a gen3_lite exists in ROS under the default loaded name, so let's use it
                    self.robot_name = "/my_gen3_lite"
                    self.degrees_of_freedom = 6
                elif rospy.has_param('/my_gen3/robot_description'):
                    # we found a gen3
                    self.robot_name = "/my_gen3"
                    self.degrees_of_freedom = 7
                else:
                    # TODO: maybe something fancier?
                    raise ValueError("Failed to find which kortex arm is specified")


        self.degrees_of_freedom = rospy.get_param(
            f"{self.robot_name}/degrees_of_freedom", self.degrees_of_freedom)
        # self.is_gripper_present = rospy.get_param(f"{self.robot_name}/is_gripper_present")
        # no defaults on these -- if these don't exist, the robot parameters are configured
        # in a way we don't understand, so we should quit
                

        rospy.loginfo(f"Using robot_name {self.robot_name}, robot has {self.degrees_of_freedom}")
                        # f" degrees of freedom and is_gripper_present is {self.is_gripper_present}")


        # Init the action topic subscriber
        self.action_topic_sub = rospy.Subscriber(
            f"{self.robot_name}/action_topic", ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        # Init a bunch of services
        # TODO: maybe do this lazily? idk if it's worth it

        # set up commonly used services
        # Any service that can be called directly *without* importing a kortex srv/msg we can
        # just alias the function to the service call

        # basic actions -- always set these up
        # TODO: can just create these through __getattr__ 
        self.clear_faults_service = rospy.ServiceProxy(
            f"{self.robot_name}/base/clear_faults", Base_ClearFaults)
        self.clear_faults = self.clear_faults_service.call
        self.stop_service = rospy.ServiceProxy(
            f"{self.robot_name}/base/stop", Stop)
        self.stop = self.stop_service.call
        self.estop_service = rospy.ServiceProxy(
            f"{self.robot_name}/base/apply_emergency_stop", ApplyEmergencyStop)
        self.estop = self.estop_service.call

        # more complex configs -- now using _service
        self.set_cartesian_reference_frame_service = rospy.ServiceProxy(
            f'{self.robot_name}/control_config/set_cartesian_reference_frame', SetCartesianReferenceFrame)


        self.read_action_service = rospy.ServiceProxy(
            f"{self.robot_name}/base/read_action", ReadAction)
        self.execute_action_service = rospy.ServiceProxy(
            f"{self.robot_name}/base/execute_action", ExecuteAction)
        self.activate_publishing_of_action_notification_service = rospy.ServiceProxy(
           f'{self.robot_name}/base/activate_publishing_of_action_topic', OnNotificationActionTopic)
        
        self.send_gripper_command_service = rospy.ServiceProxy(
            f'{self.robot_name}/base/send_gripper_command', SendGripperCommand)

        self.validate_waypoint_list_service = rospy.ServiceProxy(
            f'{self.robot_name}/base/validate_waypoint_list', ValidateWaypointList)
        
        
        # setup defult translation and orientation speed
        # when moving in cartesian space

        self.cartesian_speed = CartesianSpeed()
        self.cartesian_speed.translation = .1  # m/s
        self.cartesian_speed.orientation = 15  # deg/s

        # set up basics
        self.clear_faults()
        self.set_cartesian_reference_frame()

        # TODO: don't do this! the kortex docs say that sending this msg strains the system
        # and you shouldn't leave it active all the time
        # basic solution: make a context manager to activate/deactivate and hope there's only one at a time (or enforce)
        # complex solution: reference counting
        self.subscribe_to_a_robot_notification()

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def __getattr__(self, attr):
        """
        Autocreate services and callables if they don't exist
        """
        if attr in _KORTEX_SERVICES:
            if not hasattr(self, attr+"_service"):        
                topic, msgtype = _KORTEX_SERVICES[attr]
                service = rospy.ServiceProxy(self.robot_name + topic, msgtype)
                setattr(self, f"{attr}_service", service)
            else:
                service = getattr(self, attr+"_service")
            setattr(self, attr, service.call)
            return service.call
        elif attr.endswith("_service") and attr[:-8] in _KORTEX_SERVICES:  # -8 because '_service' is 8 characters long
            if attr.endswith("_service"):
                base_attr = attr[:-8]  # Slicing off the last 8 characters ('_service')
            else:
                base_attr = attr
            topic, msgtype = _KORTEX_SERVICES[base_attr]
            service = rospy.ServiceProxy(self.robot_name + topic, msgtype)
            # no attr was requested so just create the service
            setattr(self, attr, service)
            return service
        else:
            raise AttributeError(f"{type(self)} object has no attribute {attr}")

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

    async def action_complete(self, action, message_timeout=None):
        """
        Coroutine to block until an action is complete. TODO: use context manager to activate/deactivate
        sending these msgs

        message_timeout: Duration to wait for a notification on the action_topic topic
            NB: by default, actions only send messages at status change, so can be used as a global timeout
            for simple actions

        Raises:
            TimeoutError: No message received or rospy shutdown
            CancelledError: Action aborted or rospy shutdown
        """

        if not HAS_AIOROSPY:
            raise NotImplementedError()

        topic = f"{self.robot_name}/action_topic"
        action_event_sub = aiorospy.AsyncSubscriber(f"{self.robot_name}/action_topic", ActionNotification, 
                                                   queue_size=10) # always get newest message
        sub_iter = action_event_sub.subscribe().__aiter__()
        try:
            # let things initialize
            self.execute_action_service(action)
            while not rospy.is_shutdown():
                evt = await asyncio.wait_for(sub_iter.__anext__(), message_timeout)
                if evt.action_event == ActionEvent.ACTION_END:
                    return True
                elif evt.action_event == ActionEvent.ACTION_ABORT:
                    raise asyncio.CancelledError()
        # shut down the generator -- therefore the subscriber
        finally:
            await sub_iter.aclose()
            
        # rospy shutdown
        raise asyncio.CancelledError()

    
    async def robot_stopped(self, message_timeout=None, tolerance=0.01, stop_duration=0.25):
        """
        Catchall awaitable for robot motion
        TODO: merge with gripper motion
        """

        if not HAS_AIOROSPY:
            raise NotImplementedError()
        
        pos_sub = aiorospy.AsyncSubscriber(f"{self.robot_name}/base_feedback/joint_state", JointState, 
                                                   queue_size=1) # always get newest message
        sub_it = pos_sub.subscribe().__aiter__()
        last_tm = None
        try:
            while not rospy.is_shutdown():
                msg = await asyncio.wait_for(sub_it.__anext__(), message_timeout)
                # todo: check that velocity is also ~0?
                # TODO: actual values are very different from set values
                # send 1 -> joint ~0.95
                # send 0 -> joiny ~-0.09
                # so let's just wait for it to settle
                #
                # if np.abs(msg.position[self.degrees_of_freedom] - value) < tolerance:
                #     return True
                if np.all(np.abs(msg.velocity) <= tolerance):
                    if last_tm is None:
                        last_tm = msg.header.stamp
                    if (msg.header.stamp - last_tm).to_sec() >= stop_duration:
                        return True
                else:
                    last_tm = None
                        
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

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        #
        # TODO: implement req args
        req = OnNotificationActionTopicRequest()
        return self.activate_publishing_of_action_notification_service(req)

    def execute_action(self, action, block=True, coro=False, *coro_args, **coro_kwargs):
        # TODO: something with monitoring specific actions/actionhandles
        # TODO: make actions cancelable/pausable
        if block:
            # set up polling for notifications
            # TODO: make better
            self.subscribe_to_a_robot_notification()
            self.last_action_notif_type = None

            self.execute_action_service(action)

            return self.wait_for_action_end_or_abort()
        elif coro:
            self.subscribe_to_a_robot_notification()
            return self.action_complete(action, *coro_args, **coro_kwargs)
        else:
            self.execute_action_service(action)

    def home_arm(self, **call_args):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = Arm.HOME_ACTION_IDENTIFIER
        res = self.read_action_service(req)

        # Execute the HOME action we just read
        return self.execute_action(res.output, **call_args)

    def play_named_sequence(self, name, *args, **kwargs):
        all_seqs = self.read_all_sequences()
        seq = [ s.handle for s in all_seqs.output.sequence_list if s.name==name ]
        if len(seq) == 0:
            raise ValueError(f"Unknown sequence name: {name}")
        elif len(seq) > 1:
            raise ValueError(f"Multiple sequences corresponding to name: {name}")
        
        return self.play_sequence(seq[0], *args, **kwargs)

    def get_ik(self, pose=None, check_collisions=True):
        """
        Returns list of joints for a given pose. If no pose is give,
        will return ik for current joints. If no IK solution is found
        will return -1. If the pose is in collision, will return -2. 

        pose: list, numpy array, or PoseStamped message
        If list or numpy array, first three positions should be x,y,z position
        and next for positions should be x,y,z,w in quaternian. 
        """

        if isinstance(pose, geometry_msgs.msg.PoseStamped):
            pose_stamped = pose
        elif pose is None:
            pose = self.get_eef_pose()
            pose_stamped = geometry_msgs.msg.PoseStamped(pose)
        else:
            p, q = parse_pose_input(pose)
            pose = geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(*p),
                orientation=geometry_msgs.msg.Quaternion(*q)
            )
            pose_stamped = geometry_msgs.msg.PoseStamped(pose)

        # with moveit
        ik_req = PositionIKRequest(
            group_name="arm",
            pose_stamped=pose_stamped,
            avoid_collisions=check_collisions
        )
        ik_req.robot_state.is_diff = True

        ik_result = self.compute_ik_service(ik_req)
        if ik_result.error_code.val == -31:
            return -1
        elif ik_result.error_code.val == -12:
            return -2
        else:
            return ik_result.solution.joint_state.position[0:self.degrees_of_freedom]
            
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
        elif isinstance(joints, JointState):
            joint_state = joints
        else:
            joint_state = JointState()
            if isinstance(joints, list):
                print("get in!")
                joint_state.name = [f"joint{i+1}" for i in range(self.degrees_of_freedom)]
                joint_state.position = joints
            else:
                joint_state.name = list(joints.keys())
                joint_state.position = [joints[n] for n in joint_state.name]   # order is not technically guaranteed

        result = self.compute_fk_service(
            fk_link_names=['tool_frame'], robot_state=RobotState(joint_state=joint_state))
        
        if result.error_code.val != 1:
            return -1
        else:
            return result.pose_stamped[0]

    def get_eef_pose(self, quaternion=True):
        """
        Returns current eef pose as a PoseStamped if quaternion is True,
        otherwise returns a list of [x,y,z,theta_x,theta_y,theta_z] in radians
        """
        data = rospy.wait_for_message(
            f"{self.robot_name}/base_feedback", BaseCyclic_Feedback)
        if quaternion:
            return geometry_msgs.msg.PoseStamped(pose=kortex_pose_to_pose(data.base, prefix="tool_pose_"))
        else:
            p, r = kortex_pose_to_position_euler(data.base, prefix="tool_pose_")
            return [*p, *r]
            
    def get_joint_angles(self):
        """
        Returns current joints as a list in order of joints
        """
        joint_states = rospy.wait_for_message(
            f"{self.robot_name}/joint_states", JointState)

        return joint_states.position[:self.degrees_of_freedom]

    def set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set)
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        return self.set_cartesian_reference_frame_service(req)

    def goto_cartesian_pose(self, pose, relative=False, check_collision=False,
                            translation_speed=None, orientation_speed=None, **call_args):
        """
        Function that goes to a cartesian pose using ros_kortex's provided interface.
        pose: list, numpy array, or PoseStamped message
            If list or numpy array, can have length 3, 6, or 7.
            First three positions should be x,y,z position
            Remaining 3 are treated as euler angles or remaining for as x,y,z,w quaternion

        relative: If relative is False, the arm will go to the cartesian pose specified
        by the "pose" argument. Else if relative is true, then the the arms current cartesian 
        pose will be incremented with the passed "pose" argument.

        check_collision: If check_collision=True, the function will check if 
        the robot is in collision and return -1. If it is not in collision it will return 1.

        block: If wait_for_end=True, the function will return only after the action
        is completed or the action aborts.

        if translation_speed(m/s) or orientation_speed(deg/s) is not None, the default will be used.


        TODO: fill out the functionality of the remaining arguments
        """

        ## parse pose input
        p, q = parse_pose_input(pose)

        # build pose output
        constrained_pose = ConstrainedPose()

        if relative:
            # complicated -- we need to build the new position from the old
            # unfortunately rotation composition needs to be done with homogeneous matrices
            cur_pose = self.get_eef_pose(quaternion=False)
            cur_pose_tf = tfs.concatenate(
                tfs.translation_matrix(cur_pose[:3]),
                tfs.matrix_from_euler(*cur_pose[3:])
            )
            offset_pose_tf = tfs.concatenate(
                tfs.translation_matrix(p),
                tfs.matrix_from_quaternion(q)
            )
            target_pose = np.dot(offset_pose_tf, cur_pose_tf)
            constrained_pose.target_pose = pose_tf_to_kortex_pose(target_pose)
        else:
            constrained_pose.target_pose = pose_pq_to_kortex_pose(p, q)

        # speed
        cartesian_speed = copy.deepcopy(self.cartesian_speed)
        if translation_speed is not None:
            cartesian_speed.translation = translation_speed
        if orientation_speed is not None:
            cartesian_speed.orientation = orientation_speed
        constrained_pose.constraint.oneof_type.speed.append(cartesian_speed)

        # assemble action
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(constrained_pose)
        req.input.name = "pose"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        return self.execute_action(req, **call_args)
    
    def goto_eef_pose(self, pose, *args, **kwargs):
        """
        Function that goes to a cartesian pose using ros_kortex's provided interface.
        pose: list, numpy array, or PoseStamped message
            If list or numpy array, first three positions should be x,y,z position
            and next for positions should be x,y,z,w in quaternian. 
        TODO: add functionality for different eef frames
        """
        return self.goto_eef_waypoints([pose], *args, **kwargs)

    def goto_joint_pose(self, joints, *args, **kwargs):
        """
        Sends the arm to the specified joint angles. 
        joints: list of joint anlges (from 1 to 7 or from 1 to 6 for lite)
        radians: If true, angles will be considered in radians, 
            default is degrees.
        TODO: add dictionary functionality
        """
        return self.goto_joint_waypoints([joints], *args, **kwargs)

    def goto_zero(self, block=True):
        """
            Sends the arm fully vertical where all the joints are zero.
        """
        return self.goto_joint_pose([0 for _ in range(self.degrees_of_freedom)])

    def build_cartesian_waypoint_list(self, waypoints, blending_radius=0):
        trajectory = WaypointList()

        for pose in waypoints:
            p, q = parse_pose_input(pose)
            cart_waypoint = CartesianWaypoint(
                pose=pose_pq_to_kortex_pose(p, q),
                reference_frame=CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE, 
                blending_radius=blending_radius,
                maximum_linear_velocity=self.cartesian_speed.translation,
                maximum_angular_velocity=self.cartesian_speed.orientation
            )
            waypoint = Waypoint()
            waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cart_waypoint)
            trajectory.waypoints.append(waypoint)
        
        return trajectory
    
    def build_angular_waypoint_list(self, waypoints):
        trajectory = WaypointList()

        for i, joints in enumerate(waypoints):
            if len(joints) != self.degrees_of_freedom:
                raise ValueError(f"Unexpected length for waypoint {i}: {len(joints)}, expected {self.degrees_of_freedom}")
            waypoint = Waypoint()
            angularWaypoint = AngularWaypoint(
                angles = np.rad2deg(joints),
                duration = 0
            )
            # Initialize Waypoint and WaypointList
            waypoint.oneof_type_of_waypoint.angular_waypoint.append(
                angularWaypoint)

            trajectory.waypoints.append(waypoint)
        
        return trajectory
    
    def time_waypoint_list(self, trajectory, max_duration=30):
        ### TODO: need to do some testing if this is necessary/how to fix

        duration = 0
        while True:
            #print(duration)
            resp = self.validate_waypoint_list_service(trajectory)
            errs = resp.output.trajectory_error_report.trajectory_error_elements
            if len(errs) == 0:
                return trajectory # TODO: use optimal_waypoint_list?
            #print(errs, duration)
            # increment the duration of each waypoint to see if that helps
            duration += 0.05
            if duration > max_duration:
                # TODO: better error type
                raise RuntimeError("Duration limit exceeded when validation trajectory")
            for waypoint in trajectory.waypoints:
                waypoint.oneof_type_of_waypoint.angular_waypoint[0].duration += 0.02
    def goto_eef_waypoints(self, waypoints, blending_radius=0, duration=0, use_optimal_blending=False, **call_args):
        """
            Send the arm through a list of waypoints. 
            Each waypoint may be list, numpy array, or a list of PoseStamped messages
            TODO: add functionality for different eef frames
        """
        trajectory = self.build_cartesian_waypoint_list(waypoints, blending_radius)

        req = ExecuteActionRequest()
        req.input.duration = duration
        req.input.use_optimal_blending = use_optimal_blending
        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Call the service
        self.execute_action(req, call_args)

    def goto_joint_waypoints(self, waypoints, max_duration=30, **kwargs):
        """
        NOTE: Currently this is not functional, not sure why it does not work. 

        Sends the arm to the specified series joint angles. 
        joints: list of joint anlges (from 1 to 7)
        TODO: add dictionary functionality
        """

        req = ExecuteActionRequest()

        trajectory = self.build_angular_waypoint_list(waypoints)
        #now = time.time()
        trajectory = self.time_waypoint_list(trajectory, max_duration)
        #print(time.time() - now)
        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Send the angles
        return self.execute_action(req, **kwargs)
    def goto_joint_gripper_waypoints(self, waypoints, max_duration=30, **kwargs):
        """
        NOTE: Currently this is not functional, not sure why it does not work. 

        Sends the arm to the specified series joint angles. 
        joints: list of joint anlges (from 1 to 7)
        TODO: add dictionary functionality
        """
        print(len(waypoints))
        req = ExecuteActionRequest()
        grip_pose = waypoints[0][6]
        self.send_gripper_command(1 - grip_pose , relative=False, mode="position")
        waypoints_breakdown = []
        for i in range(len(waypoints)):
            waypoints_breakdown.append(waypoints[i][0:6])
            if abs(waypoints[i][6]  - grip_pose) > 0.2:
                print("hahha")
                
                trajectory = self.build_angular_waypoint_list(waypoints_breakdown)
                #now = time.time()
                trajectory = self.time_waypoint_list(trajectory, max_duration)
                #print(time.time() - now)
                req.input.oneof_action_parameters.execute_waypoint_list.append(
                    trajectory)
                print(len(waypoints_breakdown))
                self.execute_action(req, **kwargs)
                self.send_gripper_command(1 - waypoints[i][6], relative=False, mode="position")
                grip_pose = waypoints[i][6] 
                print(grip_pose)
                waypoints_breakdown = []
                req = ExecuteActionRequest()
            print(i)
        trajectory = self.build_angular_waypoint_list(waypoints_breakdown)
                #now = time.time()
        trajectory = self.time_waypoint_list(trajectory, max_duration)
                #print(time.time() - now)
        req.input.oneof_action_parameters.execute_waypoint_list.append(
                    trajectory)
        self.execute_action(req, **kwargs)
        self.send_gripper_command(1 -  waypoints[len(waypoints)-1][6] )
        # Send the angles
        return True
    
    def close_gripper(self, **kwargs):
        """
        Fully closes the gripper
        """
        return self.send_gripper_command(1., relative=False, mode="position", **kwargs)

    def open_gripper(self, **kwargs):
        """
        Fully opens the gripper.
        """
        return self.send_gripper_command(0., relative=False, mode="position", **kwargs)

    def send_gripper_command(self, value, relative=False, mode="position", duration=None, block=True, coro=False, **coro_args):
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
        self.send_gripper_command_service(req)
        if block:
            if mode == "position":
                rospy.sleep(1.)
            elif mode == "speed":
                rospy.sleep(duration)
        elif coro:
            if mode == "position":
                return self.gripper_command_complete(value, **coro_args)
            elif mode == "speed":
                return asyncio.sleep(duration)
    
    def get_gripper_position(self):
        """
        Returns the position of the gripper in the range 0 (open) to 1 (closed)
        """
        if self.degrees_of_freedom == 7:
            return rospy.wait_for_message(f"{self.robot_name}/base_feedback/joint_state", JointState).position[7]
        else:
            return rospy.wait_for_message(f"{self.robot_name}/base_feedback/joint_state", JointState).position[6]
        
    async def gripper_command_complete(self, value, message_timeout=1, tolerance=1e-3):
        """
        asyncio coroutine that sends a gripper command and then waits for the gripper value to match
        recommended to wrap this in an asyncio.wait_for

        value: gripper value to send
        message_timeout: duration (sec) to wait for a joint state message
        tolerance: value tolerance to say location has been reached

        raises:
            TimeoutError: if a JointState message is not received within message_timeout
            CancelledError: if rospy is shut down before goal is reached
                Note that due to timeout effects, it is indeterminate whether TimeoutError or
                CancelledError will be called if rospy is shutdown
        """
        if not HAS_AIOROSPY:
            raise NotImplementedError()
        

        gripper_pos_sub = aiorospy.AsyncSubscriber(f"{self.robot_name}/base_feedback/joint_state", JointState, 
                                                   queue_size=1) # always get newest message
        sub_it = gripper_pos_sub.subscribe().__aiter__()
        try:
            while not rospy.is_shutdown():
                msg = await asyncio.wait_for(sub_it.__anext__(), message_timeout)
                # todo: check that velocity is also ~0?
                # TODO: actual values are very different from set values
                # send 1 -> joint ~0.95
                # send 0 -> joiny ~-0.09
                # so let's just wait for it to settle
                #
                # if np.abs(msg.position[self.degrees_of_freedom] - value) < tolerance:
                #     return True
                if np.abs(msg.velocity[self.degrees_of_freedom]) <= tolerance:
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
    
 