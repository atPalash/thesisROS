#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import time
# noinspection PyUnresolvedReferences
from std_msgs.msg import Float64MultiArray
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D  # for: fig.gca(projection = '3d')

glob_curr_pos_x = None
glob_curr_pos_y = None
glob_curr_pos_z = None

try:  # Python 2/3 raw input correction
    # noinspection PyShadowingBuiltins
    input = raw_input  # overwrite the use of input with raw_input to support Python 2
except NameError:
    pass


class MotionPlanner:
    def __init__(self, visual=False, debug=False):
        """
        Initialises the MotionPlanner class with preset hard-coded calibration unless the
        automatic calibration procedure is implemented.

        :param visual: Flag control usage of matplotlib for checking results.
        :param debug: Flag for controlling extensive print statements to console.
        """

        self.visual = visual
        self.dx = 0.005
        self.debug = debug

    @staticmethod
    def discretise(point_1, point_2, dx):
        """
        Takes a straight line and divides it into smaller defined length segments.

        :param point_1: First point in 3D space
        :param point_2: Second point in 3D space
        :param dx: Distance between points in discretised line.
        :return: Numpy array of discretised line.
        """
        # create vector from point_1 to point_2
        vector = [point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]]
        # noinspection PyUnresolvedReferences
        distance = np.sqrt(sum(i ** 2 for i in vector))

        # number of points on line
        i = int(distance / dx)

        # discretise by creating new 1d array
        line_x = np.linspace(point_1[0], point_2[0], i)
        line_y = np.linspace(point_1[1], point_2[1], i)
        line_z = np.linspace(point_1[2], point_2[2], i)
        line = np.array(np.transpose(np.vstack((line_x, line_y, line_z))))
        return line

    def discretise_path(self, move, dx):
        """
        Discretise a moves path using object defined dx for unit.

        :param move: List of points path goes through.
        :param dx: Displacement between two points on the target discretised path.
        :return: Discretised path as numpy array.
        """
        move_discrete = []
        # iterate through move segments, discretise and join them
        for seg_idx in range(len(move) - 1):
            current_segment = self.discretise(move[seg_idx], move[seg_idx + 1], dx)

            # print(current_segment)
            # we add our discretised segment to our move
            if seg_idx > 0:
                # if the end of our current move is the same position as the start of our new
                # segment then we only want to add the list from the second point onwards
                if move_discrete[-1][0] == current_segment[0][0]:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment[1:]))
                else:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment))

            else:  # on first iteration, we store our segment directly
                move_discrete = current_segment
        return move_discrete

    @staticmethod
    def smooth_corners(path, size_of_corner, passes):
        """
        Takes a discretised path and and rounds the corners using parameters passed into
        function call. Minimum number of passes is 1, which results in a chamfer.

        **Note** This function is not currently used due to its error in smoothing the corners.
        It has not been implemented until a better version can be written.
        """
        if not size_of_corner % 2 == 0:  # number of steps must be an even number
            size_of_corner += 1
        steps = size_of_corner
        if passes < 1:
            raise ValueError("Number of passes must be >= 1")

        for i in range(passes):
            trajectory_1_x = [item[0] for item in path]
            trajectory_1_y = [item[1] for item in path]
            trajectory_1_z = [item[2] for item in path]

            x_tortoise = trajectory_1_x[:-steps]  # remove last few coordinates
            x_hare = trajectory_1_x[steps:]  # first last few coordinates
            x_smooth_path = [(sum(i) / 2) for i in zip(x_tortoise, x_hare)]  # average them

            y_tortoise = trajectory_1_y[:-steps]  # remove last few coordinates
            y_hare = trajectory_1_y[steps:]  # remove first few coordinates
            y_smooth_path = [(sum(i) / 2) for i in zip(y_tortoise, y_hare)]

            z_tortoise = trajectory_1_z[:-steps]  # remove last few coordinates
            z_hare = trajectory_1_z[steps:]  # remove first few coordinates
            z_smooth_path = [(sum(i) / 2) for i in zip(z_tortoise, z_hare)]

            # noinspection PyUnresolvedReferences
            smooth_path = np.array(
                [list(i) for i in zip(x_smooth_path, y_smooth_path, z_smooth_path)])
            # append first 6 coordinates in trajectory_1 to the smooth_path

            # noinspection PyUnresolvedReferences
            smooth_path = np.concatenate(
                (path[:(steps / 2)], smooth_path, path[-(steps / 2):]), axis=0)
            path = smooth_path
        return path

    @staticmethod
    def length_of_path(path):
        """
        Calculates the length of a path stored as an array of (n x 3).

        :param path: List (length n) of list (length 3) points.
        :return: The total length of the path in 3D space.
        """
        length = 0
        for i in range(len(path) - 1):
            point_a = path[i]
            point_b = path[i + 1]
            length += np.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2
                              + (point_b[2] - point_a[2]) ** 2)
        return length

    def apply_trapezoid_vel(self, path, acceleration=0.02, max_speed=0.8):
        """
        Takes a path (currently only a start/end point (straight line), and returns a discretised
        trajectory of the path controlled by a trapezium velocity profile generated by the input
        parameters.

        :param path: List of two points in 3D space.
        :param acceleration: Acceleration and deceleration of trapezium profile.
        :param max_speed: Target maximum speed of the trapezium profile.
        :return: Trajectory as numpy array.
        """
        # set rate of message sending:  0.001 sec == dt == 1kHz  NOTE THIS IS GLOBALLY SET
        dt = 0.005
        # set acceleration, start with 0.1 (may need to reduce)  NOTE THIS IS GLOBALLY SET
        acc = acceleration  # max 1.0
        # set target travel speed for motion
        target_speed = max_speed  # max 1.0

        # discretise using a max unit of:  target_speed * dt
        # ideally, we use a dx of: acc * dt**2
        dx = acc * dt ** 2  # this is the ideal delta value

        if self.debug:
            print("delta displacement (mm): ", dx * 1000)

        lop = 0
        while lop == 0:
            dis_path = self.discretise_path(path, dx)

            # SMOOTHING HAPPENS HERE
            # if smoothing is going to happen it MUST keep consistent delta displacement
            # corner = 0.05  # in meters
            # steps = int(corner * 2 / dx)
            # print("Steps: ", steps)
            # smooth_path = planner.smooth_corners(dis_path, size_of_corner=steps, passes=6)
            smooth_path = dis_path  # disable smoothing

            # find the length of the new path
            lop = self.length_of_path(smooth_path)
            if self.debug:
                print("LOP: ", lop)
            if lop == 0:
                print("Length of path is zero, adding indistinguishable offset.")
                path[1] = [c + 0.000001 for c in path[1]]

        # check if the length of path is < ( speed**2/acc )
        # this means that the length of the path is too short to accelerate all the way to the
        # target speed so we scale it down to keep a triangular profile
        minimum_path_length = target_speed ** 2 / acc
        if self.debug:
            print("Minimum path length: ", minimum_path_length)
        if lop < minimum_path_length:
            # if the length is less we need to reduce target_speed
            if self.debug:
                print("Path length is too short.")
            old_speed = target_speed
            target_speed = np.sqrt(lop * acc)
            if self.debug:
                print("Target speed changed from: ", old_speed, ", to: ", target_speed)

            # assert new target_speed is less than old for safety reasons
            assert (target_speed <= old_speed)

        else:
            # we have confirmed the length of the path is long enough for our target speed
            if self.debug:
                print("Path length ok")

        # we now need to create the speed profile graph and define its parameters

        # find t for acceleration and deceleration
        end_stage_t = target_speed / acc
        # find path distance for acc and dec
        end_stage_displacement = end_stage_t * target_speed / 2
        print("Acc/dec time: ", end_stage_t)

        # find displacement for constant speed section of motion
        mid_stage_displacement = lop - 2 * end_stage_displacement
        # find t for const speed section
        mid_stage_t = mid_stage_displacement / target_speed

        # find total time
        total_time = end_stage_t * 2 + mid_stage_t
        if self.debug:
            print("total time: ", total_time)

        # create a time list using 0->T in steps of dt
        time_list = np.arange(start=0, stop=total_time, step=dt)
        np.reshape(time_list, (np.shape(time_list)[0], 1))

        # sample speed graph to create list to go with time list
        speed_values = []
        c = (0 - (-acc) * time_list[-1])
        for t in time_list:
            if t <= end_stage_t:
                # acceleration period
                speed_values.append(acc * t)

            elif t >= end_stage_t + mid_stage_t:
                # deceleration stage
                speed_values.append(-acc * t + c)

            elif t > end_stage_t:
                # constant speed at target speed
                speed_values.append(target_speed)

        # sample path using speed list
        # noinspection PyUnboundLocalVariable
        trajectory = np.hstack((smooth_path[0, :], speed_values[0]))  # send intermediate points
        smooth_path_idx = 0

        for i in range(1, len(speed_values)):
            samples = int(np.rint(speed_values[i] * dt / dx))
            smooth_path_idx += samples
            if smooth_path_idx > len(smooth_path) - 1:
                smooth_path_idx = len(smooth_path) - 1
            new_marker = np.hstack((smooth_path[smooth_path_idx], speed_values[i]))
            trajectory = np.vstack((trajectory, new_marker))

        if self.visual:
            # plotting the board
            # plot discretised path
            # fig = plt.figure()
            # ax3d = fig.add_subplot(111, projection='3d')
            # ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')
            # ax3d.plot(smooth_path[:, 0], smooth_path[:, 1], smooth_path[:, 2], 'b*')

            # plot speed profile
            fig = plt.figure()
            # noinspection PyUnusedLocal
            ax3d = fig.add_subplot(111)
            plt.plot(time_list[:len(speed_values)], speed_values, 'r*')
            plt.ylabel("speed (m/s)")
            plt.xlabel("time (s)")

            # plot trajectory axes against time
            fig = plt.figure()
            # noinspection PyUnusedLocal
            ax3d = fig.add_subplot(111)
            plt.plot(time_list[:len(trajectory[:, 0])], trajectory[:, 0], 'r*')
            plt.plot(time_list[:len(trajectory[:, 1])], trajectory[:, 1], 'b*')
            plt.plot(time_list[:len(trajectory[:, 2])], trajectory[:, 2], 'g*')
            plt.ylabel("x(r) / y(b) / z(g) displacement (m)")
            plt.xlabel("time (s)")

            # plot trajectory in 3d
            # fig = plt.figure()
            # ax3d = fig.add_subplot(111, projection='3d')
            # ax3d.plot(path[:, 0], path[:, 1], path[:, 2], 'r')
            # ax3d.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'g*')

            plt.show()

        return trajectory


if __name__ == '__main__':
    from franka.franka_control_ros import FrankaRos

    arm = FrankaRos(init_ros_node=True)  # create an arm object for testing motion generation
    # print(arm.x, arm.y, arm.z)

    planner = MotionPlanner(visual=False, debug=True)
    #
    # EXAMPLE MOTION TO TEST IF PARTS ARE WORKING
    example_path = np.array([[arm.x, arm.y, arm.z], [-6.86302224617e-05, -0.306911210212, 0.589742275634]])  # move +y
    motion_plan = planner.apply_trapezoid_vel(example_path)
    print(motion_plan)
    arm.send_trajectory(motion_plan)
