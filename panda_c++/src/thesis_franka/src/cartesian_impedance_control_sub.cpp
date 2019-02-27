#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include <cmath>  // for isnan()
#include <limits> // for quiet_NaN

// for subscribers
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "examples_common.h"
#include <thread>

using namespace std;


// declaration of global variables
double target_x = std::numeric_limits<double>::quiet_NaN();
double target_y = std::numeric_limits<double>::quiet_NaN();
double target_z = std::numeric_limits<double>::quiet_NaN();
double joint_6_val = std::numeric_limits<double>::quiet_NaN();
double now_x = 0.0;
double now_y = 0.0;
double now_z = 0.0;
double speed = 0.0;
std::array<double, 16> current_pose_;
double target_gripper_width = 1.0;
double target_gripper_speed = 0.1;
double target_gripper_force = 60;
int gripper_grasp_stat = 0;

franka::Gripper* gripper = nullptr;  // just for the sake of creating a global object, will be changed later

/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

namespace {
    template <class T, size_t N>
    std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
        ostream << "[";
        std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
        std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
        ostream << "]";
        return ostream;
    }
}
// parameters to set
double speed_limit = 0.10;  // was tested to work properly


bool speed_below_limit(double target_speed) {
    return target_speed <= speed_limit;
}


void motionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
//  ROS_INFO("Motion callback received: [%lf, %lf, %lf, %lf]", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);  //

    target_x = msg->data[0];
    target_y = msg->data[1];
    target_z = msg->data[2];
    speed = msg->data[3];

    // new target coordinates and speed now from within the control loop
    // TODO modify x,y,z here

    // only if the wanted speed is below the speed limit is the speed adjusted
    if (speed_below_limit(speed)) {
//     speed = sub_speed;
    }
}

void graspCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ROS_INFO("Gripper callback received: width, speed, force: [%lf, %lf, %lf]", msg->data[0], msg->data[1], msg->data[2]);  //
    target_gripper_width = msg->data[0];
    target_gripper_speed = msg->data[1];
    target_gripper_force = msg->data[2];

//    gripper->homing();
    // bool moved = gripper->move(target_gripper_width, target_gripper_speed);  //  , target_gripper_force
    bool grasped = gripper->grasp(target_gripper_width, target_gripper_speed, 10, 0.1, 0.1);  //
    cout << "grasped " << grasped << endl;

    // new target coordinates and speed now from within the control loop
    // TODO modify x,y,z here

    // only if the wanted speed is below the speed limit is the speed adjusted
}

void gripperOrientCallback(const geometry_msgs::PoseStamped &msg)
{
    joint_6_val = msg.pose.orientation.z;
//    std::cout << joint_6_val << std::endl;
}


// subscriber for absolute coordinates and speed
void subscribe_target_motion() {
    // subscription stuff
    ros::NodeHandle node_handle;
    ros::Subscriber sub_motion = node_handle.subscribe("franka_move_to", 1, motionCallback);  // 1: buffer size of message queue
    ros::Subscriber sub_grasp = node_handle.subscribe("franka_gripper_grasp", 1, graspCallback);
    ros::Subscriber sub_move = node_handle.subscribe("reply_gripping_point", 1, gripperOrientCallback);
    std_msgs::Float64MultiArray states;
    states.data.resize(16);
    ros::Publisher publisher = node_handle.advertise<std_msgs::Float64MultiArray>("franka_current_position", 1);
    ros::Rate loop_rate(2000);  // 2 kHz

    while (ros::ok()) {
        for(int i = 0; i<16; i++){
            states.data[i] = current_pose_[i];
        }
        publisher.publish(states);
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "listener");

    if (argc != 2) {
        std::cerr << "Usage: ./franka_gripper_control <robot-hostname>" << std::endl;
        return -1;
    }
    try {

        franka::Robot robot(argv[1]);
        gripper = new franka::Gripper(argv[1]);
        sleep(2);
        gripper->homing();

        std::array<double, 7> q_goal = {{-1.5712350861180224, -0.3851329734534546, -0.00014185679283834537,
                                         -2.0, 0.00032089026875069563, 1.570605999765736, 0.7848645688907966}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set the collision behavior.
        std::array<double, 7> lower_torque_thresholds_nominal{
                {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
        std::array<double, 7> upper_torque_thresholds_nominal{
                {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        std::array<double, 7> lower_torque_thresholds_acceleration{
                {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
        std::array<double, 7> upper_torque_thresholds_acceleration{
                {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
        std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
        robot.setCollisionBehavior(
                lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                lower_force_thresholds_nominal, upper_force_thresholds_nominal);
        franka::Model model = robot.loadModel();
        double time = 0.0;

        auto initial_pose = robot.readOnce().O_T_EE_d;
        current_pose_ = initial_pose;

        // call the subscription thread

        std::thread t1(subscribe_target_motion);

        while(isnan(joint_6_val)){
            sleep(1);
        }
        sleep(5);
        q_goal = {{-1.5712350861180224, -0.3851329734534546, -0.00014185679283834537,
                          -2.0, 0.00032089026875069563, 1.570605999765736, 0.7848645688907966-joint_6_val}};
        MotionGenerator motion_generator1(0.5, q_goal);
        robot.control(motion_generator1);

        auto cartesian_velocity_callback = [=, &time](const franka::RobotState& robot_state,
                                 franka::Duration time_step) -> franka::CartesianVelocities {
            double vel_x = 0.0;
            double vel_y = 0.0;
            double vel_z = 0.0;

            static double old_vel_x = 0.0;
            static double old_vel_y = 0.0;
            static double old_vel_z = 0.0;

            time += time_step.toSec();

            auto state_pose = robot_state.O_T_EE_d;
            current_pose_ = state_pose;

            double cur_x = current_pose_[12];
            double cur_y = current_pose_[13];
            double cur_z = current_pose_[14];

            // initially, the robot moves to its current position (-> no motion)
            if (isnan(target_x)) {
                target_x = cur_x;
                target_y = cur_y;
                target_z = cur_z;
            }
            double vec_x = target_x - cur_x;
            double vec_y = target_y - cur_y;
            double vec_z = target_z - cur_z;

            double l2_norm = sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);

            if (l2_norm < 0.02) {
                vel_x = 0.9*old_vel_x;
                vel_y = 0.9*old_vel_y;
                vel_z = 0.9*old_vel_z;
            }
            else {
                vel_x = speed*(vec_x / l2_norm);
                vel_y = speed*(vec_y / l2_norm);
                vel_z = speed*(vec_z / l2_norm);
            }

            vel_x = 0.99*old_vel_x + 0.01*vel_x;
            vel_y = 0.99*old_vel_y + 0.01*vel_y;
            vel_z = 0.99*old_vel_z + 0.01*vel_z;

            old_vel_x = vel_x;
            old_vel_y = vel_y;
            old_vel_z = vel_z;

            franka::CartesianVelocities output = {{vel_x, vel_y, vel_z, 0.0, 0.0, 0.0}};

            double vel_norm = sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);
            return output;
        };
        // Set gains for the joint impedance control.
        // Stiffness
        const std::array<double, 7> k_gains = {{60.0, 60.0, 60.0, 60.0, 25.0, 150.0, 5.0}};
        // Damping
        const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback =
                [&model, k_gains, d_gains](
                        const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
                    // Read current coriolis terms from model.
                    std::array<double, 7> coriolis = model.coriolis(state);

                    // Compute torque command from joint impedance control law.
                    // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
                    // time step delay.
                    std::array<double, 7> tau_d_calculated;
                    for (size_t i = 0; i < 7; i++) {
                        tau_d_calculated[i] =
                                k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
                    }

                    // The following line is only necessary for printing the rate limited torque. As we activated
                    // rate limiting for the control loop (activated by default), the torque would anyway be
                    // adjusted!
                    std::array<double, 7> tau_d_rate_limited =
                            franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

                    // Send torque command.
                    return tau_d_rate_limited;
                };
        robot.control(impedance_control_callback, cartesian_velocity_callback);
        if(t1.joinable()){
            t1.join();
        }
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
