#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from objectSelection import objectSelectorOOP
from objectSelection import objectIdentifier
from reebGraph import ReebGraph
from helpers import FrameTransformation
from std_msgs.msg import Float64MultiArray
from thesis_realsense.srv import *

current_robot_pose = None


def franka_current_position(data):
    global current_robot_pose
    current_robot_pose = data.data


def detect_object_and_gripping_point(data):
    z_buffer = 0.0
    global current_robot_pose

    tf_endEffector_to_camera = [0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, -0.03, 0.05, -0.03, 1]
    tf_endEffector_to_camera = np.transpose(np.reshape(tf_endEffector_to_camera, (4, 4)))
    tf_robotBaseframe_to_end_effector = np.transpose(np.reshape(current_robot_pose, (4, 4)))
    orientation_EF = FrameTransformation.quaternion_msg_from_matrix(tf_robotBaseframe_to_end_effector)

    # print tf_robotBaseframe_to_end_effector
    object_location_wrt_camera = [[data['x']], [data['y']], [data['z']], [1]]
    # print object_location_wrt_camera
    object_location_wrt_endEffector = np.dot(tf_endEffector_to_camera, object_location_wrt_camera)

    object_location_wrt_robot_baseframe = np.dot(tf_robotBaseframe_to_end_effector, object_location_wrt_endEffector)
    object_location_wrt_robot_baseframe = {'x': object_location_wrt_robot_baseframe[0][0],
                                           'y': object_location_wrt_robot_baseframe[1][0],
                                           'z': object_location_wrt_robot_baseframe[2][0] + z_buffer}
    object_quaternion_wrt_robot_baseframe = {'x': orientation_EF.x,
                                             'y': orientation_EF.y,
                                             'z': data['o'],
                                             'w': orientation_EF.w}

    pose_goal = PoseStamped()
    pose_goal.header.stamp = rospy.Time.now()
    pose_goal.header.frame_id = "/panda_link0"
    pose_goal.pose.orientation.x = object_quaternion_wrt_robot_baseframe['x']
    pose_goal.pose.orientation.y = object_quaternion_wrt_robot_baseframe['y']
    pose_goal.pose.orientation.z = np.deg2rad(object_quaternion_wrt_robot_baseframe['z'])
    pose_goal.pose.orientation.w = object_quaternion_wrt_robot_baseframe['w']

    pose_goal.pose.position.x = object_location_wrt_robot_baseframe['x']
    pose_goal.pose.position.y = object_location_wrt_robot_baseframe['y']
    pose_goal.pose.position.z = object_location_wrt_robot_baseframe['z']

    print pose_goal

    pose_release = PoseStamped()
    pose_release.header.stamp = rospy.Time.now()
    pose_release.header.frame_id = "/panda_link0"
    pose_release.pose.orientation.x = -0.379057603396
    pose_release.pose.orientation.y = 0.923217090816
    pose_release.pose.orientation.z = 0.0418481772393
    pose_release.pose.orientation.w = 0.0472680293057

    pose_release.pose.position.x = 0.457470847753
    pose_release.pose.position.y = -0.3424774123
    pose_release.pose.position.z = -0.03100630017848

    gripper_data = GripperDataRequest()
    gripper_data.id = "objA_grasp"
    gripper_data.grasp_pose = pose_goal
    gripper_data.release_pose = pose_release
    gripper_data.max_contact_force = 5.0
    gripper_data.max_contact_velocity = 0.2
    gripper_data.max_contact_width = 0.0

    return serverRequest(gripper_data)


def serverRequest(msg):
    rospy.wait_for_service('robot_server')
    try:
        gripper_data_client = rospy.ServiceProxy('robot_server', GripperData)
        resp1 = gripper_data_client(msg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    object_list_keypress = {'objC': 97, 'objD': 98}
    # object_name = raw_input('Enter object to grasp: ')
    object_name = 'objC'

    if object_name not in object_list_keypress.keys():
        print 'No object with name found'
        exit(0)
    # gripper_data = GripperData()

    object_list = {0: 'objC', 1: 'objD'}
    selected_model = '/home/palash/thesis/thesisML/objectSelection/models/weights_best_RGB_3class.hdf5'
    object_identifier = objectIdentifier.ObjectIdentfier(selected_model, 4, 3, object_list)
    reeb_graph = ReebGraph.ReebGraph(gripper_width=1000, realtime_cam=True)
    reference_pix = (40, 40)
    padding_around_reference_pix = 10
    camera = objectSelectorOOP.RealSenseCamera(object_list=None, realsense_image_cols=1280, realsense_image_rows=720,
                                               realsense_image_padding=10, realsense_camera=True, flt_sz=3,
                                               cnny_thrsh=30,
                                               cnny_itr=10, area_threshold=1000)
    camera.set_reference_pixel(reference_pix, padding_around_reference_pix)
    camera.start_streaming()

    rospy.init_node('camera_node_for_gripping', anonymous=True)
    # gripping_point_pub = rospy.Publisher('reply_gripping_point', PoseStamped, queue_size=1)
    gripping_point_sub = rospy.Subscriber('franka_current_position', Float64MultiArray,
                                          franka_current_position)

    while True:
        if current_robot_pose is None:
            rospy.sleep(1)
            continue

        camera.get_image_data()
        images = camera.detected_object_images
        if not images:
            break
        entire_image = camera.padded_image
        cv2.imshow('entire image', entire_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        x_cord_1 = 0
        y_cord_1 = 0
        z_cord_1 = 0
        x_cord_2 = 0
        y_cord_2 = 0
        z_cord_2 = 0

        img = images[0]
        image_rgb = img['RGB']
        image_contour_xyz = img['contour']
        # got object_identifier object at 0
        #[prediction_name, prediction_percentage] = object_identifier.predict(image_rgb).split(':')
        #cv2.putText(image_rgb, prediction_name + ": " + str(prediction_percentage),
        #            (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        # cv2.imshow('detected_obj', image_rgb)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # prediction_name = "objC"
        # if object_name == prediction_name:
        reeb_graph.get_image_contour(entire_image, image_contour_xyz)
        gripping_points = reeb_graph.gripping_points
        orientation = reeb_graph.object_orientation
        for c_pts in gripping_points:
            for contour_pt in image_contour_xyz:
                if contour_pt[0][0][0] == c_pts[0][0] and contour_pt[0][0][1] == c_pts[0][1]:
                    x_cord_1 = contour_pt[1][0]
                    y_cord_1 = contour_pt[1][1]
                    z_cord_1 = contour_pt[1][2]
                if contour_pt[0][0][0] == c_pts[1][0] and contour_pt[0][0][1] == c_pts[1][1]:
                    x_cord_2 = contour_pt[1][0]
                    y_cord_2 = contour_pt[1][1]
                    z_cord_2 = contour_pt[1][2]
            cv2.circle(entire_image, c_pts[0], 1, (255, 255, 0), -1)
            cv2.circle(entire_image, c_pts[1], 1, (0, 255, 255), -1)
            cv2.putText(entire_image, "({o:.1f})".format(o=orientation), (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 1)
            cv2.putText(entire_image, "({x:.1f}, {y:.1f}, {z:.1f})".format(x=x_cord_1, y=y_cord_1, z=z_cord_1), c_pts[0],
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 1)
            cv2.putText(entire_image, "({x:.1f}, {y:.1f}, {z:.1f})".format(x=x_cord_2, y=y_cord_2, z=z_cord_2), c_pts[1],
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 1)

        cv2.imshow('entire image', entire_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        object_location = {'x': x_cord_1, 'y': y_cord_1, 'z': z_cord_1, 'o': orientation}
        print object_location
        print detect_object_and_gripping_point(object_location)
        current_robot_pose = None
        rospy.sleep(10)

# else:
#     continue
