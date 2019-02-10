#!/usr/bin/env python
import numpy as np
import rospy
import movegroupRobot
import motion_planner
from geometry_msgs.msg import PoseStamped

from franka.franka_control_ros import FrankaRos


class GripperData:
    def __init__(self):
        self.pose_goal = PoseStamped()
        self.gripper_set = False

    def set_gripper(self, position=None, orient=None):
        if position is not None and orient is not None:
            self.pose_goal.header.stamp = rospy.Time.now()
            self.pose_goal.header.frame_id = "/panda_link0"
            self.pose_goal.pose.orientation = orient
            self.pose_goal.pose.position = position
            self.gripper_set = True


def gripping_callback(data, gripper_info):
    gripper_info.set_gripper(data.pose.position, data.pose.orientation)


if __name__ == '__main__':
    arm = FrankaRos(init_ros_node=True)  # create an arm object for testing motion generation

    gripper_data = GripperData()
    gripping_point_sub = rospy.Subscriber('reply_gripping_point', PoseStamped, gripping_callback, gripper_data)
    gripping_point_pub = rospy.Publisher('find_gripping_point', PoseStamped, queue_size=1)

    # print(arm.x, arm.y, arm.z)
    if gripper_data.gripper_set:
        planner = motion_planner.MotionPlanner(visual=False, debug=True)
        example_path = np.array([[arm.x, arm.y, arm.z], [gripper_data.pose_goal.pose.x, gripper_data.pose_goal.pose.y, arm.z],
                                 [gripper_data.pose_goal.pose.x, gripper_data.pose_goal.pose.y, gripper_data.pose_goal.pose.z]])

        motion_plan = planner.apply_trapezoid_vel(example_path)

        arm.send_trajectory(motion_plan)