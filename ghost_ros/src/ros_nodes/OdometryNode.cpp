#include "ghost_ros/ros_nodes/OdometryNode.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

OdometryNode::OdometryNode() : Node("odom_node") {
    //m_current_angle_offset(0);
    m_locations = {
		Eigen::Vector2d(0.019, -3.862),
		Eigen::Vector2d(0.692, 0.703)
	};

    m_odom = m_getOdomClass(OdometryNode::OdomConfig::FOLLOWER);
    
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_imu_subscription = this->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
        "/v5/sensor_update", 10, std::bind(&OdometryNode::imuDataCallback, this, _1));

    m_odom_x_subscription = this->create_subscription<std_msgs::msg::Int32>(
        "/enc_x", 10, std::bind(&OdometryNode::odomXDataCallback, this, _1));

    m_odom_y_subscription = this->create_subscription<std_msgs::msg::Int32>(
        "/enc_y", 10, std::bind(&OdometryNode::odomYDataCallback, this, _1));
    
    m_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    m_nav_timer = this->create_wall_timer(
        5ms, std::bind(&OdometryNode::timer_callback, this));
    
    // Calibrate the gyro, and reset the orientation with the correct gyro angle
    // ROS2: Already calibrated on the V5 Side
    //m_inertial_sensor_node->reset();
	setCurrentPose(Pose(Eigen::Vector2d(0, 0), this->m_imuYaw));

    m_timer.Start();
}

IOdometry* OdometryNode::m_getOdomClass(OdomConfig config) {
    EncoderConfig adi_encoder_config = {0., 2400., 1.975};
    EncoderConfig v5_integrated_encoder_config = {0., 900., 4.0625};

    switch (config) {
        case FOLLOWER:
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config, m_locations);
        case TANK:
            return new TankOdometry(v5_integrated_encoder_config, v5_integrated_encoder_config, m_locations);
        default:
            //Node::m_handle->logerror("Error creating odometry instance in OdometryNode.cpp");
            return new FollowerOdometry(adi_encoder_config, adi_encoder_config, m_locations);
    }
}

void OdometryNode::timer_callback(){
    Pose previousPose = m_odom->GetPose();

    // Update Odometry
    m_odom->Update(this->m_odomXTicks, this->m_odomYTicks, this->m_imuYaw);
    
    double currentTime = m_timer.Get();
    double deltaTime = currentTime - lastTime;

    Pose currentPose = m_odom->GetPose();
    
    // Create quaternion for later use
    tf2::Quaternion q;
    q.setRPY(0, 0, currentPose.angle.angle());
    
    // Create and fill odom->base_link transform
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    
    t.transform.translation.x = currentPose.position.x();
    t.transform.translation.y = currentPose.position.y();
    t.transform.translation.z = 0.0;
    
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    // Publish odom->base_link transform
    m_tf_broadcaster->sendTransform(t);
    
    // Create and fill odom message
    auto odom_msg = nav_msgs::msg::Odometry();
    
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Pose portion
    odom_msg.pose.pose.position.x = currentPose.position.x();
    odom_msg.pose.pose.position.y = currentPose.position.y();
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // Twist portion
    odom_msg.twist.twist.linear.x = (currentPose.position.x() - previousPose.position.x()) / deltaTime;
    odom_msg.twist.twist.linear.y = (currentPose.position.y() - previousPose.position.y()) / deltaTime;
    odom_msg.twist.twist.linear.z = 0.0;
    
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = (currentPose.angle.angle() - previousPose.angle.angle()) / deltaTime;

    // Publish odom message
    m_publisher->publish(odom_msg);
}

void OdometryNode::imuDataCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg) {
    float yawRadians = msg->encoders[ghost_v5_config::IMU_SENSOR].angle_degrees * -(M_PI/180);
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Angle: " << yawRadians);
    Eigen::Rotation2Dd current_angle(msg->encoders[ghost_v5_config::IMU_SENSOR].angle_degrees * -(M_PI/180));
    this->m_imuYaw = current_angle;
}

void OdometryNode::odomXDataCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    this->m_odomYTicks = msg->data;
}

void OdometryNode::odomYDataCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    this->m_odomYTicks = msg->data;
}

void OdometryNode::setCurrentPose(Pose pose) {
    m_odom->SetCurrentPose(pose);
}

Pose OdometryNode::getCurrentPose() {
    return m_odom->GetPose();
}

/*void OdometryNode::teleopPeriodic() {
    m_odom->Update(this->m_odomXTicks, this->m_odomYTicks, this->m_imuYaw);
    
    Pose currentPose = m_odom->GetPose();
    float x = currentPose.position.x();
    float y = currentPose.position.y();
    float angle = currentPose.angle.angle();

    //pros::lcd::print(0, "Position: (%.2f, %.2f)\n", x, y);
    //pros::lcd::print(1, "Angle: %.2f", angle);

    // if (m_timer.Get() - lastTime > delayTime) {
    //     std::cout << "X Pos: " << m_odom->GetPose().position.x() << " | Y Pos: " << m_odom->GetPose().position.y() << " | Angle: " << m_odom->GetPose().angle.angle() << std::endl;
    //     lastTime = m_timer.Get();
    // }
}

void OdometryNode::autonPeriodic() {
    m_odom->Update(this->m_odomXTicks, this->m_odomYTicks, this->m_imuYaw);

    Pose currentPose = m_odom->GetPose();
    float x = currentPose.position.x();
    float y = currentPose.position.y();
    float angle = currentPose.angle.angle();
    
    //pros::lcd::print(0, "Position: (%.2f, %.2f)\n", x, y);
    //pros::lcd::print(1, "Angle: %.2f", angle);
}*/

OdometryNode::~OdometryNode() {
    delete m_odom;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}