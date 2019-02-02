#!/usr/bin/env python
import os
import sys

import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonRobot:
    """MoveGroupPythonJointRecorder"""

    def __init__(self):
        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_joint_value_recorder', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % self.planning_frame

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % self.eef_link

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()

        self.current_joint_val = []
        self.proceed_to_next_joint = False

    def current_coordinate_values(self):
        return self.move_group.get_current_pose()


    def current_joint_values(self):
        return self.move_group.get_current_joint_values()

    def go_to_state(self, joint_val=None, cartesian_val=None):
        if (joint_val is None and cartesian_val is None) or (joint_val is not None and cartesian_val is not None):
            print 'enter either joint value or cartesian values'
        if joint_val is not None:
            self.initial_joint_values = joint_val
            return self.move_group.go(joint_val, wait=True)
        if cartesian_val is not None:
            return self.go_to_pose_goal(cartesian_val)

    def go_to_joint_state(self, joint_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)
        rospy.sleep(2)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, cartesian_val):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = cartesian_val['orientation']['w']
        pose_goal.orientation.x = cartesian_val['orientation']['x']
        pose_goal.orientation.y = cartesian_val['orientation']['y']
        pose_goal.orientation.z = cartesian_val['orientation']['z']
        pose_goal.position.x = cartesian_val['position']['x']
        pose_goal.position.y = cartesian_val['position']['y']
        pose_goal.position.z = cartesian_val['position']['z']

        move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        rospy.sleep(2)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_next(self, data):
        self.proceed_to_next_joint = True

    def wait_for_snap(self):
        self.proceed_to_next_joint = False

    @staticmethod
    def write_data(record_path, rec_joint_vals, append=False):
        try:
            file = os.listdir(record_path)
            filenum = len(file)
            if append:
                filenum = filenum - 1
            record_file = open(record_path + '/' + record_path + str(filenum) + '.txt', 'a')
            np.savetxt(record_file, rec_joint_vals, delimiter=',', fmt='%f')
        except:
            print 'error csv'


if __name__ == '__main__':
    robot = MoveGroupPythonRobot()
    go_to = {'position':{'x': -0.05199, 'y': 0.5493, 'z': 0.26701}, 'orientation':{
        'x': 0.2249, 'y':0.97433, 'z':0.00850287, 'w': 0.001963}}
    print robot.current_coordinate_values()
    robot.go_to_state(joint_val=None, cartesian_val=go_to)