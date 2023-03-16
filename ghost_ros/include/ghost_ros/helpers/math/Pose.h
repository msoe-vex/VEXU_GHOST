#pragma once

#include "eigen3/Eigen/Dense"

struct Pose {
    Eigen::Vector2d position;
    Eigen::Rotation2Dd angle;

    Pose(Eigen::Vector2d positionIn, Eigen::Rotation2Dd angleIn) {
        position = positionIn;
        angle = angleIn;
    }

    Pose() {
        position = Eigen::Vector2d(0, 0);
        angle = Eigen::Rotation2Dd(0);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};