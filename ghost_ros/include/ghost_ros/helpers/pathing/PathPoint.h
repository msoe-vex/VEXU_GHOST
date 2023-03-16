#pragma once

#include "eigen3/Eigen/Dense"
#include "ghost_ros/helpers/math/Pose.h"

class PathPoint {
public:
    PathPoint(float time, Pose pose, Eigen::Vector2d linear_velocity, float rotational_velocity);

    float getTime();

    Pose getPose();

    Eigen::Vector2d getLinearVelocity();

    float getRotationalVelocity();

    PathPoint interpolateTo(PathPoint other, float time);

    bool equals(PathPoint* that);
    
private:
    Pose m_pose;
    float m_time;
    Eigen::Vector2d m_linear_velocity;
    float m_rotational_velocity;
};