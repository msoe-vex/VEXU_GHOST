#pragma once

#include <vector>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "ghost_ros/helpers/pathing/PathPoint.h"
#include "ghost_ros/helpers/math/Pose.h"

class Path {
public:
    Path();

    Path(std::vector<PathPoint> pathPoints);

    Pose update(float time);

    std::vector<PathPoint> getPathPoints();

    bool isComplete();

private:
    std::vector<PathPoint> m_pathPoints;
    PathPoint m_last_point;
    bool m_is_complete;
};