#pragma once

#include <eigen3/Eigen/Dense>

class RobotParams {
    public:
        RobotParams();

        double mass;
        Eigen::Matrix3d inertia;
};