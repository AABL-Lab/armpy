import os
import rospy

# hardcoding robotiq for now, if we get a different hand we can make it smarter
from robotiq_85_msgs.msg import GripperCmd, GripperStat


class Gripper:
    def __init__(self, prefix='right'):

        self.pub_grp = rospy.Publisher(
            '/gripper/cmd', GripperCmd, queue_size=10)
        self.sub_grp = rospy.Subscriber('gripper/stat', GripperStat, self.st_cb)

        # Wait for a connection to the gripper.
        while self.pub_grp.get_num_connections() < 1:
            rospy.loginfo_throttle(
                5, "Gripper wrapper waiting for connection to driver...")
            rospy.sleep(0.05)
        rospy.loginfo("Gripper wrapper connected.")

        self.cmd = GripperCmd()

        # i have it here but it is not useful
        #rospy.Subscriber('/vector/right_gripper/joint_states', JointState, self.js_cb)
        #self.last_js_update = None
        #self.joint_state = None

        self.gripper_stat = GripperStat()

    # def js_cb(self, inState):
    #  self.joint_state = inState.position
    #  self.last_js_update = rospy.get_time()

    def st_cb(self, inStat):
        ### TODO(rma): make this thread-safe. ros is single-threaded, but if something else (e.g. a gui) accesses this variable it may be unhappy.
        self.gripper_stat = inStat

    def is_ready(self):
        return self.gripper_stat.is_ready

    def is_reset(self):
        return self.gripper_stat.is_reset

    def is_moving(self):
        return self.gripper_stat.is_moving

    def object_detected(self):
        return self.gripper_stat.obj_detected

    def get_pos(self):
        return self.gripper_stat.position

    def get_commanded_pos(self):
        return self.gripper_stat.requested_position

    def get_applied_current(self):
        return self.gripper_stat.current

    def set_pos(self, position, block=True, speed=0, force=0):
        position = max(min(position, 1.0), 0.0)
        speed = max(min(speed, 255), 0)
        force = max(min(force, 255), 0)
        if not block:
            rospy.logwarn('Moving the gripper asynchronously. This can be dangerous. Pass block=True '
                          'to make this function wait until the gripper is done moving.')
        self.cmd.position = position
        self.cmd.speed = speed
        self.cmd.force = force
        self.pub_grp.publish(self.cmd)
        rospy.sleep(1.0)
        if block:
            # TODO(Kukanani): this really shouldn't be an arbitrary sleep length.
            #                 Instead, the driver should monitor the status flags
            #                 returned by the Robotiq to determine when to return.
            rospy.sleep(3 - (speed / 255))

    def open(self, block=True, speed=255, force=0):
        self.set_pos(1, block, speed, force)

    def close(self, block=True, speed=255, force=0):
        self.set_pos(0, block, speed, force)
