#pragma once

#include "eigen3/Eigen/Dense"
#include "ghost_ros/helpers/math/Pose.h"

class Waypoint {
private:
    Pose m_position;
    Eigen::Vector2d m_velocity;
    float m_time;

public:
    Waypoint(Pose position, Eigen::Vector2d velocity, float time);

    Pose getPosition();

    Eigen::Vector2d getVelocity();

    float getTime();
};