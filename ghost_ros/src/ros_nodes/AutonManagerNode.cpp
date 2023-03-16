#include "ghost_ros/ros_nodes/AutonManagerNode.h"

//namespace rr_ros {
    AutonManagerNode::AutonManagerNode(std::vector<Auton*> autons) : 
        rclcpp::Node("auton_manager"), m_autons(autons) {
            selected_auton = m_autons.at(0);
            pathJSON = "/usd/paths/path.json";

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
        PathManager::GetInstance()->LoadPathsFile(pathJSON);
    }

    void AutonManagerNode::setPathsFile(std::string filename) {
        pathJSON = "/usd/paths/" + filename;
        PathManager::GetInstance()->LoadPathsFile(pathJSON);
    }

    void AutonManagerNode::autonPeriodic() {
        if(!selected_auton->Complete()) {
            selected_auton->AutonPeriodic();
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
//}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::vector<Auton*> empty_autons_vec;
    rclcpp::spin(std::make_shared<AutonManagerNode>(empty_autons_vec));
    rclcpp::shutdown();
    return 0;
}