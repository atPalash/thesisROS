//
// Created by palash on 27.2.2019.
//
#include <math.h>
#include <assert.h>
#include "MotionPlanner.h"

MotionPlanner::MotionPlanner(double dx) {
}

MotionPlanner::~MotionPlanner() = default;

template<typename T>
std::vector<double> linspace(T &start_in, T &end_in, double num_in)
{

    std::vector<double> linspaced;

    auto start = static_cast<double>(start_in);
    auto end = static_cast<double>(end_in);
    auto num = static_cast<double>(num_in);

    if (num == 0.0) { return linspaced; }
    if (num == 1.0)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1.0);
    linspaced.resize(static_cast<unsigned long>(num));
    for(int i=0; i < num-1; ++i)
    {
        linspaced[i] = start + delta * i;
    }
    linspaced[linspaced.size()-1] = end; // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

std::vector<std::vector<double>> MotionPlanner::discretise(std::vector<double> &point1, std::vector<double> &point2, double dx) {
    std::vector<double> p1_to_p2;
    p1_to_p2.push_back({point2[0] - point1[0]});
    p1_to_p2.push_back({point2[1] - point1[1]});
    p1_to_p2.push_back({point2[0] - point1[1]});

    double distance = sqrt(pow(p1_to_p2[0],2) + pow(p1_to_p2[1], 2) + pow(p1_to_p2[2], 2));

    int num_discrete_points = (int)(distance/dx);

    std::vector<double> line_x = linspace(point1[0], point2[0], num_discrete_points);
    std::vector<double> line_y = linspace(point1[1], point2[1], num_discrete_points);
    std::vector<double> line_z = linspace(point1[2], point2[2], num_discrete_points);

    std::vector<std::vector<double>> line;
    line.resize(num_discrete_points);

    for(int i=0;i < num_discrete_points; i++){
//        std::vector<double > cur_point;
//        cur_point.resize(3);
//        cur_point.push_back(line_x[i]);
//        cur_point.push_back(line_y[i]);
//        cur_point.push_back(line_z[i]);
        line[i] = {line_x[i], line_y[i], line_z[i]};
    }
    return line;
}

std::vector<std::vector<double>> MotionPlanner::discretisePath(std::vector<std::vector<double>> &waypoints, double dx) {
    std::vector<std::vector<double>> move_discrete;

    for(int i=0; i<waypoints.size()-1; i++){
        std::vector<std::vector<double>> current_segment = discretise(waypoints[i], waypoints[i+1], dx);
        if(i > 0){
            if(move_discrete[move_discrete.size()-1][0] == current_segment[0][0]) {
                current_segment.erase(current_segment.begin());
            }
            move_discrete.insert(move_discrete.end(), current_segment.begin(), current_segment.end());
        } else{
            move_discrete.assign(current_segment.begin(), current_segment.end());
        }
    }
    return move_discrete;
}

double MotionPlanner::lengthOfPath(std::vector<std::vector<double>> &path) {
    double length = 0.0;
    for(int i=0; i<path.size() -1; i++){
        std::vector<double> point_a = path[i];
        std::vector<double> point_b = path[i+1];

        length += sqrt(pow((point_b[0] - point_a[0]), 2) + pow((point_b[1] - point_a[1]), 2) +
                       pow((point_b[2] - point_a[2]), 2));
    }
    return length;
}

std::vector<std::vector<double>> MotionPlanner::applyTrapezoidalVelocity(std::vector<std::vector<double>> path,
                                                                         double acceleration, double max_speed) {
    double dt = 0.005;
    double acc = acceleration;
    double target_speed = max_speed;

    double dx = acc*pow(dt, 2);

    double count = 0.0;
    std::vector<std::vector<double>> dis_path;
    while(count == 0.0){
        dis_path = discretisePath(path, dx);
        count = lengthOfPath(dis_path);
    }

    double minimum_path_length = pow(target_speed, 2) / acc;

    if(count < minimum_path_length){
        double old_speed = target_speed;
        target_speed = sqrt(count*acc);
        assert(target_speed <= old_speed);
    }
    double end_stage_t = target_speed / acc;
    double end_stage_displacement = end_stage_t * target_speed / 2;

    double mid_stage_displacement = count - 2*end_stage_displacement;
    double mid_stage_t = mid_stage_displacement / target_speed;

    double total_time = end_stage_t*2 + mid_stage_t;
    double t1 = 0.0;
    std::vector<double> time_list = linspace(t1, total_time, total_time/dt);
    std::cout << time_list.size() << std::endl;
    std::vector<double> speed_values;
    speed_values.resize(time_list.size());
    double c = acc*time_list[time_list.size() - 1];
    int itr = 0;
    for(auto t:time_list){
        if(t<=end_stage_t){
            speed_values[itr] = acc*t;
        } else if(t >=end_stage_t + mid_stage_t){
            speed_values[itr] = -acc*t + c;
        } else if(t>end_stage_t){
            speed_values[itr] = target_speed;
        }
        itr++;
    }

    std::vector<std::vector<double>> trajectory;
    trajectory.resize(speed_values.size());
    trajectory[0] = {dis_path[0][0], dis_path[0][1], dis_path[0][2], speed_values[0]};
    int smooth_path_idx = 0;
    for(int i=1; i<speed_values.size();i++){
        int samples = (int)(round(speed_values[i]*dt/dx));
        smooth_path_idx += samples;
        if(smooth_path_idx > dis_path.size() -1){
            smooth_path_idx = (int)dis_path.size() - 1;
        }
        trajectory[i] = {dis_path[smooth_path_idx][0], dis_path[smooth_path_idx][1], dis_path[smooth_path_idx][2], speed_values[i]};
    }
    return trajectory;
}


