#!/usr/bin/env python

from logging import error
from queue import Empty
import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
import geometry_msgs.msg
import std_msgs.msg
import kinova_msgs.srv
from math import pi, floor, ceil, fabs

class Arm:
    """ MoveIt! wrapper for planning for the HLP-R robot's arm(s).

    This class provides a simplified interface to the planning capabilities of
    the MoveIt! ROS packages.  To plan to a specific joint configuration::
    
        a = ArmMoveit()
        pose = [pi,pi,pi,pi]
    
        a.goto_pose(pose,is_joint_pos=True)

    """

    def __init__(self, planning_frame='base_link', eef_frame='j2s7s300_ee_link', default_planner="RRTConnectkConfigDefault"):
        """ Create an interface to ROS MoveIt with a given frame and planner.

        Creates an interface to ROS MoveIt!. Right now this only creates
        a planner for a single arm. The default values of the arguments
        should work well for most applications; look into the MoveIt! 
        documentation if you think you'd like to change them.

        **IMPORTANT:** this class maps all commands to the continuous joints
        into (-pi,pi).  For some applications this may result in unexpected
        or undesirable behavior!

        Parameters
        ----------
        planning_frame : str, optional_simplify
            the frame in which MoveIt! should plan
        eef_frame : str, optional
            the end effector frame for pose goals
        default_planner : str, optional
            which planner to use with MoveIt!
        """

        # Make sure the moveit service is up and running
        rospy.logwarn("Waiting for MoveIt! to load")
        try:
            rospy.wait_for_service('compute_ik')
        except rospy.ROSException as e:
            rospy.logerr("No MoveIt service detected. Exiting")
            exit()
        else:
            rospy.loginfo("MoveIt detected: arm planner loading")

        # self.pose = geometry_msgs.msg.PoseStamped()

        # Check if we're using the 7dof
        if 'VECTOR_HAS_KINOVA_7DOF_ARM' in os.environ:
            is_7dof = os.environ['VECTOR_HAS_KINOVA_7DOF_ARM']
        else:
            is_7dof = True

        ## Interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
        
        ## Interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Array of interfaces to single groups of joints.  
        ## At the moment, only a single arm interface is instantiated
        self.group = moveit_commander.MoveGroupCommander("arm")
        print(self.group.get_end_effector_link())

        ## The name of the planner to use
        self.planner = default_planner

        # Set the planning pose reference frame
        self.group.set_pose_reference_frame(planning_frame)
        self.group.set_end_effector_link(eef_frame)

        ## The names of continuous joints. They will always be remapped
        ## into (-pi,pi)
        if is_7dof:
            self.continuous_joints = ['joint_1','joint_3','joint_5','joint_7']
        else:
            self.continuous_joints = ['shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']

    def home_arm(self):
        """
        Homes the arm. 
        """
        rospy.wait_for_service("/j2s7s300_driver/in/home_arm")
        home_service = rospy.ServiceProxy("/j2s7s300_driver/in/home_arm", kinova_msgs.srv.HomeArm)
        home_service()

    def emergency_stop(self):
        """
        Emergency stop the arm
        """
        rospy.wait_for_service("/j2s7s300_driver/in/stop")
        stop_arm = rospy.ServiceProxy("/j2s7s300_driver/in/stop", kinova_msgs.srv.Stop)
        stop_arm()


    def open_gripper(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 1
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def close_gripper(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def start_force_control(self):
        """
        Calls the start force control service.
        """
        rospy.wait_for_service("/j2s7s300_driver/in/start_force_control")
        force_service = rospy.ServiceProxy("/j2s7s300_driver/in/start_force_control", kinova_msgs.srv.Start)
        force_service()
        print("Force control has been enabled")
    
    def stop_force_control(self):
        """
        Calls the stop force control service.
        """
        rospy.wait_for_service("/j2s7s300_driver/in/stop_force_control")
        force_service = rospy.ServiceProxy("/j2s7s300_driver/in/stop_force_control", kinova_msgs.srv.Stop)
        force_service()
    
    def set_velocity(self, velocity=0.2):
        """
        Alters the speed of trajectory

        Parameters
        ----------
        velocity : float
            Allowed values are in (0,1]
        """
        if velocity > 0 and velocity <= 1:
            self.group.set_max_velocity_scaling_factor(velocity)
        else:
            raise Exception("Expected value in the range from 0 to 1 for scaling factor" )

    def set_acceleration(self, acceleration=0.1):
        """
        Alters the acceleration of trajectory

        Parameters
        ----------
        acceleration : float
            Allowed values are in (0,1]
        """
        if acceleration > 0 and acceleration <= 1:
            self.group.set_max_acceleration_scaling_factor(acceleration)
        else:
            raise Exception("Expected value in the range from 0 to 1 for scaling factor" )
        
    def get_IK(self, new_pose = None, root = None, avoid_collisions=False):
        """ Find the corresponding joint angles for an end effector pose
        
        Uses MoveIt! to compute the inverse kinematics for a given end
        effector pose.

        Parameters
        ----------
        new_pose : geometry_msgs/PoseStamped 
            the end effector pose (if none is provided, uses the current pose)
        root : string, optional
            the root link (if none is provided, uses the planning frame)

        Returns
        ----------
        list 
            The joint angles corresponding to the end effector pose
        """
        
        ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

        pose = (self.get_FK(root=root) if new_pose is None else new_pose)
        try:
            pose = copy.deepcopy(pose)[0]
        except:
            pose = copy.deepcopy(pose)

        msgs_request = moveit_msgs.msg.PositionIKRequest()
        msgs_request.group_name = self.group.get_name()
        msgs_request.pose_stamped.pose = pose
        msgs_request.robot_state.is_diff = True
        msgs_request.timeout.secs = 2
        msgs_request.avoid_collisions = avoid_collisions
        msgs_request.ik_link_names = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4",
                        "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]

        # msgs_request.robot_state = self.robot.get_current_state()
        try:
            jointAngle=compute_ik(msgs_request)
            ans=list(jointAngle.solution.joint_state.position)[2:9]
            ans = self._simplify_joints(ans)
            if jointAngle.error_code.val == -31:
                print('No IK solution')
                return None
            if (jointAngle.error_code.val == -12 or jointAngle.error_code.val==-10) and avoid_collisions is True:
                raise ValueError("Goal or current position is in collision")
            return ans
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        
    def get_FK(self, root = None, state = None):
        """ Find the end effector pose for the current joint configuration
        
        Uses MoveIt! to compute the forward kinematics for a given joint
        configuration.

        Parameters
        ----------
        root : string, optional
            the root link (if none is provided, uses the planning frame)
        state : RobotState, optional
            the state to calculate FK for (if none is provided, uses the 
                current state)
        
        Returns
        ----------
        geometry_msgs/PoseStamped []
            the pose of the end effector
        """
        rospy.wait_for_service('compute_fk')
        compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

        header = std_msgs.msg.Header()
        header.frame_id = root
        header.stamp = rospy.Time.now()
        # THE CHANGE:
        fk_link_names = ['j2s7s300_link_7']
        robot_state = self.robot.get_current_state()
        #print(robot_state.joint_state.position) 
        #joints = dict(zip(robot_state.joint_state.name[2:9], robot_state.joint_state.position[2:9]))
        #joints = self._simplify_joints(joints)
        #print(robot_state.joint_state.position[2:9])
        #robot_state.joint_state.position[2:9] = joints.values()
        try:
            reply=compute_fk(header,fk_link_names,robot_state)
            return reply.pose_stamped

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def check_arm_collision(self, joints = None):
        '''Gets whether a given joint of the arm pose is in collision 
        with an object in the scene or not. If no joints
        are provided, checks if the current pose is in collision.
        
        Parameters
        ----------
        joints : list or dictionary, optional
            If not provided, uses current joint pos.
        Returns
        ----------
        bool
            True if arm is in collision. False otherwise.
        '''
        rospy.wait_for_service('/check_state_validity')
        collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        if joints is None:
            joints = self.get_current_pose()

        robot_state = self.state_from_joints(joints)

        validityRequest = GetStateValidityRequest()
        validityRequest.robot_state=robot_state
        
        collisions = collison_service(validityRequest)
        
        for i in range(len(collisions.contacts)):
            body_1 = collisions.contacts[i].contact_body_1
            body_2 = collisions.contacts[i].contact_body_2
            if "j2" in body_1 or "j2" in body_2:
                return True
        
        return False

    def get_arm_collisions(self, joints = None):
        '''Returns a list of collisions with the arm given a joint pose. 
        If no joints are provided, checks if the current pose 
        is in collision.
        
        Parameters
        ----------
        joints : list or dictionary, optional
            If not provided, uses current joint pos.
        Returns
        ----------
        list
            list of tuples containing arm collisions.
        '''
        rospy.wait_for_service('/check_state_validity')
        collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        if joints is None:
            joints = self.get_current_pose()

        robot_state = self.state_from_joints(joints)

        validityRequest = GetStateValidityRequest()
        validityRequest.robot_state=robot_state
        
        collisions = collison_service(validityRequest)
        
        collision_list = []
        for i in range(len(collisions.contacts)):
            body_1 = collisions.contacts[i].contact_body_1
            body_2 = collisions.contacts[i].contact_body_2
            if "j2" in body_1 or "j2" in body_2:
                collision_list.append((body_1, body_2))
        
        return collision_list

    def check_robot_collision(self, joints = None):
        '''Gets whether any part of the robot is currently in collision 
        or not. Optionally can provide joints. 
        
        Parameters
        ----------
        joints : list or dictionary, optional
            If not provided, uses current joint pos.
        Returns
        ----------
        bool
            True on success
        '''
        rospy.wait_for_service('/check_state_validity')
        collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        if joints is None:
            joints = self.get_current_pose()

        robot_state = self.state_from_joints(joints)

        validityRequest = GetStateValidityRequest()
        validityRequest.robot_state=robot_state
        
        collisions = collison_service(validityRequest)
        
        if len(collisions.contacts) == 0:
            return False
        return True

    def get_robot_collisions(self, joints = None):
        '''Returns a list of collisions with the robot given a joint pose. 
        If no joints are provided, checks if the current pose 
        is in collision.
        
        Parameters
        ----------
        joints : list or dictionary, optional
            If not provided, uses current joint pos.
        Returns
        ----------
        list
            list of tuples containing robot collisions.
        '''
        rospy.wait_for_service('/check_state_validity')
        collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        if joints is None:
            joints = self.get_current_pose()

        robot_state = self.state_from_joints(joints)

        validityRequest = GetStateValidityRequest()
        validityRequest.robot_state=robot_state
        
        collisions = collison_service(validityRequest)
        
        collision_list = []
        for i in range(len(collisions.contacts)):
            body_1 = collisions.contacts[i].contact_body_1
            body_2 = collisions.contacts[i].contact_body_2
            collision_list.append((body_1, body_2))
        
        return collision_list

    def plan_joint_pos(self, target, starting_config=None):
        """ Plan a trajectory to reach a given joint configuration
        
        Uses MoveIt! to plan a trajectory to reach a given joint
        configuration

        Parameters
        ----------
        target : list or dict
            if a list, a list of positions for all active joints 
            in the group; if a dictionary, a mapping of joint names 
            to positions for the joints that should be moved (all other 
            joints will be held at their current angles).
        starting_config : RobotState, optional
            the starting configuration to plan from.  If not set, will 
            be set to the current state of the robot.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        """
        
        self.set_start_state(starting_config)
        self.set_joint_target(target)
        return self.get_plan()

    def plan_ee_pos(self, target, starting_config=None):
        """ Plan a trajectory to reach a given end effector position
        
        Uses MoveIt! to plan a trajectory to reach a given end effector
        position

        Parameters
        ----------
        target : geometry_msgs/Pose or geometry_msgs/PoseStamped
            the desired pose of the end effector
        starting_config : RobotState, optional
            the starting configuration to plan from.  If not set, will 
            be set to the current state of the robot.

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        """
        self.set_start_state(starting_config)
        self.set_ee_target(target)
        return self.get_plan()

    def set_start_state(self, joint_config=None):
        """ Set the starting state from which to plan
        
        Sets the MoveIt! starting position in preparation for planning

        Parameters
        ----------
        joint_config (RobotState) -- the starting configuration to plan
            from.  If not set, will be set to the current state of the robot.
        """

        if joint_config is not None:
            start_state = joint_config
        else:
            start_state = self._copy_state()

        try:
            self.group.set_start_state(start_state)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set start state: {}'.format(e))


    def set_joint_target(self, target):
        """ Set the joint configuration target
        
        Sets the MoveIt! target position in preparation for planning

        Parameters
        ----------
        target (list or dict) -- if a list, a list of positions for
            all active joints in the group; if a dictionary, a mapping
            of joint names to positions for the joints that should be
            moved (all other joints will be held at their current angles).
        """
        try:
            self.group.set_joint_value_target(self._simplify_joints(target))
            self.group.set_planner_id(self.planner)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set target and planner: {}'.format(e))

    def set_ee_target(self, target):
        """ Set the end effector position target
        
        Sets the MoveIt! target position in preparation for planning

        Parameters
        ----------
        target : geometry_msgs/Pose
            the desired pose of the end effector
        """
        
        try:
            # sanitize possible numpy before sending off to moveit
            if type(target).__module__ == 'numpy':
                target = target.tolist()

            self.group.set_pose_target(target)
            self.group.set_planner_id(self.planner)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('Unable to set target and planner: {}'.format(e))

    def get_plan(self):
        '''Generates a plan for reaching the current goal
        
        Uses MoveIt! to generate a plan based on the previously-set starting
        position and target position.

        .. note:: You must set the target and starting position before calling
            this function.
        
        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        '''
        try:
            plan =  self.group.plan()[1]
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr('No plan found: {}'.format(e))
            return None
        return plan

    def move_robot(self, plan, wait = True):
        '''Moves the robot according to a plan
        
        Uses MoveIt! to move the arm according to the provided plan.

        .. warning:: The plans can be stitched together, but this can 
            have unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        plan : RobotTrajectory
            a plan generated by MoveIt!
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True

        Returns
        ----------
        bool
            True on success
        '''
        return self.group.execute(plan, wait)




    def plan_joint_waypoints(self, targets, starting_config=None):
        '''Generates a multi-segment plan to reach waypoints in joint space
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can 
            have unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of joint targets (either list or dict; see documentation 
            for :py:meth:`plan_joint_pos` )
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        all_plans = []
        current_config = starting_config
        for target in targets:
            plan = self.plan_joint_pos(target, starting_config=current_config)
            if plan!=None:
                all_plans.append(plan)
                try:
                    current_config=self.state_from_trajectory(plan.joint_trajectory)
                except moveit_commander.MoveItCommanderException as e:
                    rospy.logerr("Couldn't set configuration. Error:{}".format(e))
        return all_plans

    def plan_ee_waypoints(self, targets, starting_config=None):
        '''Generates a multi-segment plan to reach end effector waypoints
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can have 
            unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of geometry_msgs/Pose for the end effector
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        all_plans = []
        plan_start_config = starting_config
        for target in targets:
            plan = self.plan_ee_pos(target, plan_start_config)
            if plan is not None:
                if len(plan.joint_trajectory.points) != 0:
                    all_plans.append(plan)
                    plan_start_config = self.state_from_trajectory(
                        plan.joint_trajectory)
            else:
                rospy.logerr('EE waypoints could not calculate plan')
        return all_plans

    def go_to_joint_state(self, target, grip_pos = None, grip = None, wait=True):
        # move to the target joint configuration
        
        success = self.group.go(target, wait=wait)
        if grip_pos is not None:
            if grip_pos > 0.8:
                grip.close()
            else:
                grip.open()
        return success

    def go_to_joint_states(self, targets, grip_pos = None, grip = None, wait=True):
        # move to the target joint configuration

        
        if grip_pos is not None:
            for i in range(len(targets)):
                print(i)
                success = self.go_to_joint_state(targets[i], grip_pos[i], grip, wait)
                # go back to reduce the error
                if not success:
                    i -= 2
                    print("failed")
        else:
            for i in range(len(targets)):
                print(i)
                success = self.go_to_joint_state(targets[i], None, None, wait)
                # go back to reduce the error
                if not success:
                    i -= 2
                    print("failed")
            




    def move_through_waypoints(self, targets, grip_pos = None, grip = None, is_joint_pos=False, wait=True):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of either joint positions or end effector positions. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True

        Returns
        ----------
        bool
            True on success
        '''
        cnt = 0
     
        plans = self.plan_waypoints(targets, is_joint_pos)
        if plans == None or len(plans)==0:
            rospy.logwarn('no plans generated')
            return False
        success = True
        for idx, plan in enumerate(plans):
            cnt+=1
            print(cnt)
            success = success and self.move_robot(plan, wait)
            # unfortunate hack. There appears to be a significant settling time
            # for the kinova arm.
            if grip_pos is not None:
                if grip_pos[cnt] > 0.8:
                    grip.close()
                else:
                    grip.open()

            if not is_joint_pos and idx < (len(plans)-1):
                rospy.sleep(0.5)
        return success

    def move_through_joint_waypoints(self, targets):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of joint positions (either list or dict)
            See the documentation for plan_joint_waypoints.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_through_waypoints(targets, is_joint_pos=True)


    def move_to_joint_pose(self, target):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified joint pose, 
        then moves the robot. Returns True on success, otherwise returns False.

        Parameters
        ----------
        target : list or dict
            a joint position (list or dict). See the documentation for 
            plan_joint_waypoints.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_to_pose(target, is_joint_pos=True)

    def move_to_ee_pose(self, target):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified joint pose, 
        then moves the robot. Returns True on success, otherwise returns False.

        Parameters
        ----------
        target : geometry_msgs/Pose
            an end effector position. See the documentation for 
            plan_ee_waypoints.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_to_pose(target, is_joint_pos=False)
    
    def move_through_ee_waypoints(self, targets):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot through the specified end effector 
        or joint pose waypoints, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        targets : list
            a list of end effector positions (geometry_msgs/Pose)
            See the documentation for plan_ee_waypoints.

        Returns
        ----------
        bool
            True on success
        '''
        return self.move_through_waypoints(targets, is_joint_pos=False)    

    def move_to_pose(self, target, is_joint_pos=False, wait=True):
        '''Uses MoveIt! to generate a plan then move the robot.

        Generates a plan to move the robot to the specified end effector 
        or joint pose, then moves the robot. Returns True on success,
        otherwise returns False.

        Parameters
        ----------
        target : list, dict, or geometry_msgs/Pose
            either a joint position (list or dict) or end effector position. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        wait : bool, optional
            whether to return immediately or block until movement is complete. 
            Default value is True

        Returns
        ----------
        bool
            True on success
        '''
        plan = self.plan_pose(target, is_joint_pos=is_joint_pos)
        if plan != None:
            return self.move_robot(plan, wait)
        else:
            return False
    
    def plan_pose(self, target, is_joint_pos=False, starting_config=None):
        '''Plan a trajectory to reach a given end effector or joint pose

        Uses MoveIt! to plan a trajectory to reach a given end effector
        position or joint configuration.

        Parameters
        ----------
        target : list, dict, or geometry_msgs/Pose
            either a joint position (list or dict) or end effector position. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)

        Returns
        ----------
        RobotTrajectory
            the plan for reaching the target position
        '''
        if is_joint_pos:
            return self.plan_joint_pos(target, starting_config)
        else:
            return self.plan_ee_pos(target, starting_config)

    def plan_waypoints(self, targets, is_joint_pos=False
                       ,merge_plans=False, starting_config=None):
        '''Generates a multi-segment plan to reach waypoints in target space
        
        Uses MoveIt! to generate a plan from target to target. One plan is
        generated for each segment.

        .. warning:: The plans can be stitched together, but this can have 
            unexpected issues since the plan boundaries can violate 
            acceleration limits.
        
        Parameters
        ----------
        targets : list
            a list of either joint positions or end effector positions. 
            See the documentation for plan_joint_waypoints and
            plan_ee_waypoints.
        is_joint_pos : bool, optional
            True if the targets are given in joint space (defaults to False)
        merge_plans : bool
            if True, return a single merged plan (see note above about 
            potential issues)
        starting_config : RobotState
            the starting state. If not provided this will default to the 
            current state of the robot.

        Returns
        ----------
        list of RobotTrajectory
            the plan for reaching the target position
        '''
        if is_joint_pos:
            print('joint pos in plan_waypoints')
            plans = self.plan_joint_waypoints(targets, starting_config)
        else:
            plans = self.plan_ee_waypoints(targets, starting_config)
        
        if merge_plans:
            return self._merge_plans(plans)
        else:
            return plans

    def get_current_pose(self, simplify=True, is_dict=True):
        '''Returns the current pose of the planning group
        
        Parameters
        ----------
        simplify : bool, optional
            whether or not to simplify continuous joints into +/- pi

        Returns
        ----------
        dict
            the joint positions, mapped into (-pi,pi) if simplify
        '''
        if not is_dict:
            return self._simplify_joints(self.group.get_current_joint_values())
        if simplify:
            return dict(zip(self.group.get_active_joints(),self._simplify_joints(self.group.get_current_joint_values())))
        else:
            return dict(zip(self.group.get_active_joints(),self.group.get_current_joint_values())) 
        
    def get_current_ee_pose(self, simplify=True):
        """
        Returns the current pose of the end effector of the planning group.

        Parameters:
        simplify (bool, optional): Whether or not to simplify the pose of continuous joints into +/- pi.

        Returns:
        A tuple containing the x, y, and z position of the end effector in meters.
        """


        # Check if the simplify flag is set
        if simplify:
            # Get the current pose of the end effector and simplify it
            ee_pose = self.group.get_current_pose()
            return ee_pose
        else:
            # If the simplify flag is not set, do nothing and return None
            pass



    def get_planning_frame(self):
        '''
        Return
        ---------
        string
            the frame used to initialize the planner
        '''
        return self.group.get_pose_reference_frame()

    def get_eef_frame(self):
        '''
        Return
        ---------
        string
            the frame used to initialize the end-effector link
        '''
        
        return self.group.get_end_effector_link()

    def state_from_joints(self, joints):
        ''' Returns a RobotState object with given joint positions

        Returns a RobotState object with the given joints
        set to the given positions.  The joints may be given
        as a dict or list object.  All other joints are taken from
        the current state.

        Parameters
        ----------
        joints : list or dict
            if a list, a list of positions for all active joints in 
            the group; if a dictionary, a mapping of joint names to 
            positions for the joints that should be moved (all other 
            joints will be held at their current angles).

        Returns
        ----------
        RobotState
            A RobotState object with only the given joints changed
        '''
        state = self._copy_state()
        #print('joints: ' + str(type(joints)))
        simple_joints = self._simplify_joints(joints)
        if isinstance(joints, dict):
            joint_names = joints.keys()
            new_joints = [x for x in state.joint_state.position]
            
            for jname in joint_names:
                new_joints[state.joint_state.name.index(jname)]=simple_joints[jname]
            state.joint_state.position = new_joints
        elif isinstance(joints, list):
            state.joint_state.position = copy.copy(joints)
        else:
            rospy.logerr("Joints must be provided as a list or dictionary")
            raise TypeError("Joints must be provided as a list or dictionary")
        return state

    def state_from_trajectory(self, trajectory, point_idx=-1):
        ''' Returns a RobotState object with joint positions from trajectory

        Returns a RobotState object with joint positions taken from the 
        given trajectory. By default, sets the position to the last point
        in the trajectory.

        Parameters
        ----------
        trajectory : JointTrajectory
            the trajectory from which to take the joint positions.
        point_idx : int
            which point in the trajectory to take the state from. Defaults 
            to -1, taking the last point.

        Returns
        ----------
        RobotState 
            A RobotState object with state corresponding to the
            given point in the trajectory.
        '''

        state = self._copy_state()
        target = trajectory.points[point_idx]
        joints = [x for x in state.joint_state.position]
        for i in range(len(trajectory.joint_names)):
            joint_name = trajectory.joint_names[i]
            idx = state.joint_state.name.index(joint_name)
            joints[idx]=target.positions[i]
        state.joint_state.position = joints
        return state

    def _simplify_angle(self, angle):
        # Very simple function that makes sure the angles are between -pi and pi
        if angle > pi:
            while angle > pi:
                angle -= 2*pi
        elif angle < -pi:
            while angle < -pi:
                angle += 2*pi

        return angle

    def _simplify_joints(self, joints):
        # Helper function to convert a dictionary of joint values
        if isinstance(joints, dict):
            simplified_joints = dict()
            for joint in joints:
                # Pull out the name of the joint
                joint_name = '_'.join(joint.split('_')[1::])
                if joint_name in self.continuous_joints:
                    simplified_joints[joint] = self._simplify_angle(joints[joint])
                else:
                    simplified_joints[joint] = joints[joint]
        elif isinstance(joints, list):
            simplified_joints = []
            #separate the joint name from the group name
            joint_order = [ "_".join(s.split("_")[1::]) for s in self.group.get_active_joints() ]
            
            continuous_joint_indices = [joint_order.index(j) for j in self.continuous_joints]

            for i in range(len(joints)):
                a = joints[i]
                if i in continuous_joint_indices:
                    simplified_joints.append(self._simplify_angle(a))
                else:
                    simplified_joints.append(a)
        else:
            rospy.logerr("Joints must be provided as a list or dictionary")
            raise TypeError("Joints must be provided as a list or dictionary")
        return simplified_joints

    def _copy_state(self):
        ''' Copies the robot state (so it can be modified to plan from
            non-current joint configurations).'''
        ## output: a copy of the robot's current state
        current_state = self.robot.get_current_state()
        for name, state in zip(current_state.joint_state.name, current_state.joint_state.position):
            print("%s %.2f" % (name, state))
        return copy.deepcopy(self.robot.get_current_state())

    def _merge_plans(self, plan_list, time_between=0.1):
        #check if the list is empty
        if plan_list==None or len(plan_list)==0:
            rospy.logwarn("Cannot merge plans: no plans provided")
            return plan_list
        
        all_points = []
        start_time = rospy.Duration(0)
        for plan in plan_list:
            for point in plan.joint_trajectory.points:
                new_point = copy.deepcopy(point)
                new_point.time_from_start = new_point.time_from_start+start_time
                all_points.append(new_point)
            start_time=all_points[-1].time_from_start+rospy.Duration(time_between)
                
        full_plan = copy.deepcopy(plan_list[0])
        full_plan.joint_trajectory.points=all_points
        return full_plan


    ''' Deprecated function definitions maintained for backwards compatibility '''

    def plan_targetInput(self, target, joint_flag):
        '''***DEPRECATED*** Generic target planner that what type is specified'''
        if (joint_flag):
            self.plan_joint_pos(target)
        else:
            self.plan_ee_pos(target)

    def plan_targetInputWaypoint(self, targets, joint_flag, merged=False, 
                                 current_joints=None):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        return self.plan_waypoints(targets, joint_flag, 
                                   merged, current_joints)

    def set_robot_state_pose(self, traj):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        return self.state_from_trajectory(traj, point_idx=-1)

    def set_robot_state_joint_dict(self, joint_dict):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        return self.state_from_joints(joint_dict)
    
    def get_active_joints(self):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        return self.group.get_active_joints()

    def merge_points(self, points, new_points, time_between=0.1):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        # Check if this is the first set
        if len(points) < 1:
            return new_points

        all_points = points
        # Pull out the last time from current points
        last_point_time = points[-1].time_from_start+rospy.Duration(time_between)
        for point in new_points:
            point.time_from_start = point.time_from_start+last_point_time
            all_points = all_points + [point]
        return all_points

    def plan_jointTargetInput(self,target_joint):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        ## input: target joint angles (list) of the robot
        ## output: plan from current joint angles to the target one
        return self.plan_joint_pos(target_joint)

    def plan_poseTargetInput(self,target_pose):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        ## input: tart pose (geometry_msgs.msg.Pose())
        ## output: plan from current  pose to the target one
        return self.plan_ee_pos(target_pose)
        
    def box_table_scene(self):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        #Scene : add box 
        # after loading this object/scene, need to do "add" ==> "planning scene"  
        # in the rviz interface if one want to see the box
        
        rospy.sleep(2)
        self.scene.remove_world_object("table_box")
        
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 1.64
        p.pose.position.y = 0.0
        p.pose.position.z = 0.25
        p.pose.orientation.w = 0.0
        self.scene.add_box("table_box",p,(0.75, 1, 0.5))
     
        rospy.sleep(5)

    def wayPointIK(self, wps, numSteps = None, ik_root = None):
        '''***DEPRECATED*** Left in for backwards compatibility'''
        if numSteps is None:
            numSteps = 3
            jointWps = []

        for i in range(0, len(wps)):
            jointP = self.get_IK(wps[i], ik_root)
            if jointP is None:
                jointWps = None
                break
            jointWps.append(jointP)

        return jointWps

def ask_scene_integration(arm):
    # Ask the user if want to integrate a box scene
    answer= input("""\n Integrate a box as a table from code ? (1 or 0)  
  (if 1: box can't be displaced nor resized by user, if 0: no scene (can always do add from rviz interface) ) \n""")
    
    if answer == 1:
        arm.box_table_scene()
        print("\n Box inserted; to see it ==> rviz interface ==> add button==> planning scene  ")
        return
    else:
        print("\n No scene added")
        return  
      
def ask_position(arm,tarPose):
    #Ask the user the values of the target position
    while True:
        try:   
            inputPosition=input(""" \n Target position coord. (format: x,y,z or write -1 to take the robot current position ): """)      
            
            if inputPosition == -1:
                inputPosition=tarPose.position  
                return inputPosition
          
        except (ValueError,IOError,NameError):
            print("\n Please, enter the coordinate in the following format: x,y,z ")
            continue
        else:          
            if len(list(inputPosition)) == 3:
                poseTmp= geometry_msgs.msg.Pose()
                poseTmp.position.x=inputPosition[0]
                poseTmp.position.y=inputPosition[1]
                poseTmp.position.z=inputPosition[2]
                return poseTmp.position
            else:
                print("\n Please, enter the coordinate in the following format: x,y,z ")
                continue
            
        
def ask_orientation(arm,tarPose):
    # Ask the user the values of the target quaternion
    while True:
        try:   
            inputQuat=input(""" \n Target quaternion coordi. (format: qx,qy,qz,qw or write -1 to take the robot current quaternion ):""")
            
            if inputQuat == -1:
                inputQuat=arm.group.get_current_pose().pose.orientation                   
            return  inputQuat
          
        except (ValueError,IOError,NameError):
            print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
            continue
        else:
            if len(list(inputQuat)) == 4:
                poseTmp= geometry_msgs.msg.Pose()
                poseTmp.orientation.x=inputQuat[0]
                poseTmp.orientation.y=inputQuat[1]
                poseTmp.orientation.z=inputQuat[2]
                poseTmp.orientation.w=inputQuat[3]
                return poseTmp.orientation    
            else:
                print("\n Please, enter the coordinate in the following format: qx,qy,qz,qw ")
            
def main():
    arm = Arm()

    tarPose = geometry_msgs.msg.Pose()

    ## ask if integrate object scene from code or not
    ask_scene_integration(arm)
    
    while not rospy.is_shutdown():
      
        ##   Assigned tarPose the current Pose of the robot 
        tarPose = arm.group.get_current_pose().pose
      

        ## ask input from user (COMMENT IF NOT USE AND WANT TO ASSIGN MANUAL VALUE IN CODE)    
        tarPose.position = ask_position(arm,tarPose)   
        tarPose.orientation = ask_orientation(arm,tarPose)  

        ##  Example of Assigned values for new targetPose to robot
        #    tarPose.position.x = 0.89
        #    tarPose.position.y = 0.00
        #    tarPose.position.z = 0.32   
        #    tarPose.orientation.x = 0.0     
   
        print('\n The target coordinate is: %s \n' %tarPose )
    
        ## IK for target position  
        jointTarg = arm.get_IK(tarPose)
        print('IK calculation step:DONE' )
    
        ## planning with joint target from IK 
        planTraj =  arm.plan_jointTargetInput(jointTarg)
        print('Planning step with target joint angles:DONE')
    
        ## planning with pose target
        #print 'Planning step with target pose'   
        #planTraj = arm.plan_poseTargetInput(tarPose)
    
        ## execution of the movement   
        #print 'Execution of the plan' 
        # arm.group.execute(planTraj)
    
if __name__ == '__main__':
    ## First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('vector_basic_IK', anonymous=True)
    
    main()

