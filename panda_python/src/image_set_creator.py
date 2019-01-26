#!/usr/bin/env python
import rospy
import movegroupRobot
import numpy as np
import moveit_commander
from std_msgs.msg import String

def generate_robot_motion(record_path):
    try:
        recorder = movegroupRobot.MoveGroupPythonRobot()
        buffer_size = 100
        buffer_joint_vals = np.zeros((buffer_size, 7), dtype=float)
        count1 = 0
        count2 = 0
        rate = rospy.Rate(100)
        append_to_file = False
        try:
            prev_val = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            while not rospy.is_shutdown():
                joint_val_recorded = recorder.current_joint_values()
                if len(joint_val_recorded) is not 7:
                    print 'continue'
                    continue
                else:
                    diff = False
                    for itr in range(len(joint_val_recorded)):
                        if abs(joint_val_recorded[itr] - prev_val[itr]) > 0.1:
                            diff = True
                            break
                    if diff:
                        prev_val = joint_val_recorded
                        buffer_joint_vals[count1, :] = joint_val_recorded
                        print "write to buffer", count2, ':', buffer_joint_vals[count1, :]
                        if count1 is buffer_size - 1:
                            recorder.write_data(record_path, buffer_joint_vals, append_to_file)
                            count1 = -1
                            append_to_file = True
                            print 'write to txt'
                        count2 = count2 + 1
                        count1 = count1 + 1
                        rate.sleep()
        except KeyboardInterrupt:
            print 'data collected'
        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def follow_trained_trajectory(filename):
    follower = movegroupRobot.MoveGroupPythonRobot()

    ready_for_snap_pub = rospy.Publisher('ready_for_snap', String, queue_size=1)
    snap_taken_subscriber = rospy.Subscriber('snap_taken', String, follower.go_to_next)
    rate = rospy.Rate(10)  # 10hz

    file_joint_val = open(filename, "r")
    lines = file_joint_val.readlines()
    for line in lines:
        try:
            joint_values = [float(x) for x in line.split(',')]
            try:
                follower.go_to_state(joint_val=joint_values, cartesian_val=None)
                rospy.sleep(1)
                follower.wait_for_snap()
                ready_for_snap_msg = "ready"
                while not rospy.is_shutdown():
                    if follower.proceed_to_next_joint:
                        break
                    ready_for_snap_pub.publish(ready_for_snap_msg)
                    rate.sleep()

            except moveit_commander.move_group.MoveItCommanderException:
                print 'moveit Exception'
                continue
        except ValueError:
            print 'joint values not properly formed'
            continue


if __name__ == '__main__':
    path_to_save_joint_values = 'recorded_joint_val'
    # generate_robot_motion(path_to_save_joint_values)

    joint_values_to_follow_txt = path_to_save_joint_values + '/' + path_to_save_joint_values + str(2) + '.txt'
    follow_trained_trajectory(joint_values_to_follow_txt)

