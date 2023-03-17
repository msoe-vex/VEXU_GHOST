#include "ghost_ros/ros_nodes/AutonManagerNode.h"

namespace rr_ros {
    AutonManagerNode::AutonManagerNode() : rclcpp::Node("auton_manager") {
        // Load ROS Params (default json file)
        declare_parameter("auton_path_default", "/usd/paths/paths.json");
        std::string auton_path_default = get_parameter("auton_path_default").as_string();
        PathManager::GetInstance()->LoadPathsFile(auton_path_default);
        RCLCPP_DEBUG(get_logger(), "Default Auton Path: " + auton_path_default);

        competition_state_sub_ = create_subscription<ghost_msgs::msg::V5CompetitionState>(
            "v5/competition_state",
            10,
            [this](const ghost_msgs::msg::V5CompetitionState::SharedPtr msg) {
                curr_comp_state_msg_id_ = msg->msg_id;
                curr_comp_state_msg_ = msg;
                
                if (!msg->is_autonomous && timer_ != nullptr) {
                    timer_->cancel();
                }

                if (msg->is_autonomous && curr_robot_state_ != robot_state_e::AUTONOMOUS) {
                    curr_robot_state_ = robot_state_e::AUTONOMOUS;
                    if (timer_ == nullptr) {
                        timer_ = create_wall_timer(5ms, std::bind(&AutonManagerNode::periodic, this));
                    } else {
                        timer_.reset();
                    }
                }
                else if (msg->is_disabled) {
                    curr_robot_state_ = robot_state_e::DISABLED;
                }
                else if (!msg->is_autonomous && !msg->is_disabled) {
                    curr_robot_state_ = robot_state_e::TELEOP;
                }
            }
        );
    }

    void AutonManagerNode::initialize() {
    }

    void AutonManagerNode::setAutons(std::vector<Auton*> autons) {
        m_autons = autons;
        m_selected_auton = m_autons.at(0);
    }

    std::vector<Auton*> AutonManagerNode::getAutons() {
        return m_autons;
    }
    
    void AutonManagerNode::setSelectedAuton(int index) {
        m_selected_auton = m_autons.at(index);
    }
    
    Auton* AutonManagerNode::getSelectedAuton() {
        return m_selected_auton;
    }
    
    void AutonManagerNode::autonPeriodic() {
        if(!m_selected_auton->Complete()) {
            m_selected_auton->AutonPeriodic();
        }
    }
    
    void AutonManagerNode::periodic() {
        // Run autonPeriodic or teleopPeriodic based on competition state
        switch (curr_robot_state_) {
            case robot_state_e::AUTONOMOUS:
                autonPeriodic();
                break;

            case robot_state_e::TELEOP:
                break;

            case robot_state_e::DISABLED:
                break;
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rr_ros::AutonManagerNode>());
    rclcpp::shutdown();
    return 0;
}