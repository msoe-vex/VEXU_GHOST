#pragma once

#include "ghost_ros/helpers/math/Pose.h"
#include "ghost_ros/helpers/math/Math.h"
#include "ghost_ros/helpers/util/Encoders.h"
#include "ghost_ros/helpers/util/Constants.h"

class IOdometry {
public:
    struct EncoderLocations {
        Eigen::Vector2d x_encoder_location;
        Eigen::Vector2d y_encoder_location;
    };

    IOdometry(EncoderConfig encoder_1_config, EncoderConfig encoder_2_config, EncoderLocations locations, Pose current_pose=Pose());

    void ResetEncoderTicks(double encoder_1_ticks=0, double encoder_2_ticks=0);

    Pose GetPose();

    void SetCurrentPose(Pose current_pose);

    virtual void Update(double encoder_1_raw_ticks, double encoder_2_raw_ticks, double track_width) {}
    virtual void Update(double encoder_1_raw_ticks, double encoder_2_raw_ticks, Eigen::Rotation2Dd gyro_angle) = 0;

    ~IOdometry();

protected:
    Eigen::Rotation2Dd m_gyro_initial_angle;

    Pose m_robot_pose = Pose(Eigen::Vector2d(0, 0), Eigen::Rotation2Dd());

    bool m_pose_reset = true;

    double m_encoder_1_ticks_to_dist;
    double m_encoder_2_ticks_to_dist;

    double m_last_encoder_1_dist;
    double m_last_encoder_2_dist;

    Eigen::Rotation2Dd m_gyro_offset = Eigen::Rotation2Dd(GYRO_OFFSET);
};