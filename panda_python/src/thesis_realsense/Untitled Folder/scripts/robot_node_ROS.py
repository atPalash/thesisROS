#!/usr/bin/env python
import numpy as np
import rospy
import motion_planner
from geometry_msgs.msg import PoseStamped

from franka.franka_control_ros import FrankaRos
from panda_python.srv import *
import rospy
import time


class GripperPose:
    def __init__(self):
        self.pose_goal = PoseStamped()
        self.gripper_set = False
        self.motion_plan = False

    def set_gripper(self, position=None, orient=None):
        if position is not None and orient is not None:
            self.pose_goal.header.stamp = rospy.Time.now()
            self.pose_goal.header.frame_id = "/panda_link0"
            self.pose_goal.pose.orientation = orient
            self.pose_goal.pose.position = position
            self.gripper_set = True
            self.motion_plan = True


def gripping_callback(data, gripper_info):
    gripper_info.set_gripper(data.pose.position, data.pose.orientation)


def handle_add_two_ints(req):
    print req.goal.pose.position.x
    # return req.a + req.b


def robot_server():
    rospy.init_node('add_two_ints_server')
    arm = FrankaRos(init_ros_node=False)
    s = rospy.Service('add_two_ints', GripperData, handle_add_two_ints)

    # gripper_pose = GripperPose()
    # gripper_pose.pos
    # print "Ready to add two ints. / plan"
    # planner = motion_planner.MotionPlanner(visual=False, debug=True)
    # path = np.array(
    #     [[arm.x, arm.y, arm.z], [gripper_pose.pose_goal.pose.position.x, gripper_pose.pose_goal.pose.position.y,
    #                              gripper_pose.pose_goal.pose.position.z]])
    # motion_plan = planner.apply_trapezoid_vel(path)
    # arm.send_trajectory(motion_plan)
    # arm.grasp(0.001, 0.5, 0.9)
    # example_path = np.array([[arm.x, arm.y, arm.z], [arm.x, arm.y, 0.3]])  # move +y
    # motion_plan = planner.apply_trapezoid_vel(example_path)
    # arm.send_trajectory(motion_plan)
    rospy.spin()


if __name__ == '__main__':
    robot_server()