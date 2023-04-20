#pragma once

#include <math.h>
#include "ghost_ros/helpers/odometry/IOdometry.h"
#include "ghost_ros/helpers/util/Constants.h"
#include "ghost_ros/helpers/util/Logger.h"

class FollowerOdometry : public IOdometry {
private:
    EncoderLocations m_locations;
    bool m_previously_positive = true;

public:
    FollowerOdometry(EncoderConfig x_encoder_config, EncoderConfig y_encoder_config, EncoderLocations locations, Pose current_pose=Pose());
    
    void Update(double x_encoder_raw_ticks, double y_encoder_raw_ticks, Eigen::Rotation2Dd gyro_angle);

    ~FollowerOdometry();
};