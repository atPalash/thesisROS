//
// Created by palash on 27.2.2019.
//

#ifndef INTERVIEWPRAC_MOTIONPLANNER_H
#define INTERVIEWPRAC_MOTIONPLANNER_H

#include <iostream>
#include <vector>

class MotionPlanner {
public:
    MotionPlanner(double dx);
    ~MotionPlanner();
    std::vector<std::vector<double >> discretise(std::vector<double> &point1, std::vector<double > &point2, double dx);
    std::vector<std::vector<double >> discretisePath(std::vector<std::vector<double>> &waypoints, double dx);
    double lengthOfPath(std::vector<std::vector<double>> &path);
    std::vector<std::vector<double >> applyTrapezoidalVelocity(std::vector<std::vector<double >> path,
                                                               double acceleration = 0.02, double max_speed = 0.8);
};


#endif //INTERVIEWPRAC_MOTIONPLANNER_H
