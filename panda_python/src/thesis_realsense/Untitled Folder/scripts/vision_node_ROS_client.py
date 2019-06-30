#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from panda_python.srv import *


def serverRequest(msg):
    rospy.wait_for_service('robot_server')
    try:
        add_two_ints = rospy.ServiceProxy('robot_server', GripperData)
        resp1 = add_two_ints(msg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    rospy.init_node('franka_client')

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/panda_link0"
    # goal.pose.position.x = 0.306900861674
    # goal.pose.position.y = -7.92872696268e-05
    # goal.pose.position.z = 0.59046834332
    #
    # goal.pose.orientation.x = -0.923840182035
    # goal.pose.orientation.y = 0.382778386448
    # goal.pose.orientation.z = -0.000129596558825
    # goal.pose.orientation.w = 9.01705604783e-05

    goal.pose.position.x = -0.306900861674
    goal.pose.position.y = -0.506911210212
    goal.pose.position.z = 0.289742275634

    goal.pose.orientation.x = -0.38278353548
    goal.pose.orientation.y = 0.923837689903
    goal.pose.orientation.z = 0.000539873037812
    goal.pose.orientation.w = 0.000629458136054

    gripper_data = GripperDataRequest()
    gripper_data.id = "objA_grasp"
    gripper_data.grasp_pose = goal
    gripper_data.max_contact_force = 0.5
    gripper_data.max_contact_velocity = 0.2
    gripper_data.max_contact_width = 0.1

    # goal.pose.position.x = -6.86302224617e-05
    # goal.pose.position.y = -0.306911210212
    # goal.pose.position.z = 0.589742275634
    #
    # goal.pose.orientation.x = -0.38278353548
    # goal.pose.orientation.y = 0.923837689903
    # goal.pose.orientation.z = 0.000539873037812
    # goal.pose.orientation.w = 0.000629458136054
    #
    # gripper_data.release_pose = goal

    print serverRequest(gripper_data)
