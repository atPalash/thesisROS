#!/usr/bin/env python
import rospy
import movegroupRobot
from geometry_msgs.msg import PoseStamped


def gripping_callback(data):
    print data


if __name__ == '__main__':
    # rospy.init_node('robot_node_for_gripping', anonymous=True)
    gripping_point_sub = rospy.Subscriber('reply_gripping_point', PoseStamped, gripping_callback)
    gripping_point_pub = rospy.Publisher('find_gripping_point', PoseStamped, queue_size=1)

    robot = movegroupRobot.MoveGroupPythonRobot()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        robot_current_state = robot.current_coordinate_values()
        gripping_point_pub.publish(robot_current_state)
        rate.sleep()
