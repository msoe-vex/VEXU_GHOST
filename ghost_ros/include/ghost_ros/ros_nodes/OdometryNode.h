#pragma once

#include "rclcpp/rclcpp.hpp"

//#include "lib-rr/nodes/sensor_nodes/ADIEncoderNode.h"
//#include "lib-rr/nodes/sensor_nodes/InertialSensorNode.h"
//#include "lib-rr/nodes/actuator_nodes/MotorNode.h"

#include "ghost_ros/robot_config/v5_port_config.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "ghost_ros/helpers/odometry/IOdometry.h"
#include "ghost_ros/helpers/odometry/FollowerOdometry.h"
#include "ghost_ros/helpers/odometry/TankOdometry.h"
#include "ghost_ros/helpers/util/Encoders.h"
#include "eigen3/Eigen/Dense"
#include "ghost_ros/helpers/util/Logger.h"
#include "ghost_ros/helpers/util/Timer.h"

class OdometryNode : public rclcpp::Node { 
public:
    enum OdomConfig {
        FOLLOWER, TANK
    };
    
    OdometryNode();
    
    void timer_callback();

    void imuDataCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);

    void odomXDataCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void odomYDataCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void setCurrentPose(Pose pose);

    Pose getCurrentPose();

    void teleopPeriodic();

    void autonPeriodic();

    ~OdometryNode();

private:    
    OdomConfig m_odom_config;

    IOdometry::EncoderLocations m_locations;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr m_imu_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_odom_x_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_odom_y_subscription;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_nav_timer;
    
    Eigen::Rotation2Dd m_imuYaw;
    long m_odomXTicks = 0;
    long m_odomYTicks = 0;

    //ADIEncoderNode* m_odom_encoder_1;
    //ADIEncoderNode* m_odom_encoder_2;
    //MotorNode* m_motor_1;
    //MotorNode* m_motor_2;
    //InertialSensorNode* m_inertial_sensor_node;
    Eigen::Rotation2Dd m_current_angle_offset;

    IOdometry* m_odom;

    Timer m_timer;
    double lastTime = 0.;
    double delayTime = 0.25;

    IOdometry* m_getOdomClass(OdomConfig odom_config);
};
