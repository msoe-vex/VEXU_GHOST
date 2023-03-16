#pragma once

#include "ghost_ros/helpers/pathing/PathManager.h"
#include "ghost_ros/helpers/auton/Auton.h"
#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "ghost_msgs/msg/v5_competition_state.hpp"
#include "rclcpp/rclcpp.hpp"

//namespace rr_ros {
    enum robot_state_e
    {
        DISABLED = 0,
        TELEOP = 1,
        AUTONOMOUS = 2,
    };

    class AutonManagerNode : public rclcpp::Node {
    private:
        std::string pathJSON;


        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Subscription<ghost_msgs::msg::V5CompetitionState>::SharedPtr competition_state_sub_;

        ghost_msgs::msg::V5CompetitionState::SharedPtr curr_comp_state_msg_;
        uint32_t curr_comp_state_msg_id_;

        ghost_msgs::msg::GhostRobotState::SharedPtr curr_robot_state_msg_;
        uint32_t curr_robot_state_msg_id_;
        robot_state_e curr_robot_state_;

        void periodic();

    public:
        AutonManagerNode(std::vector<Auton*> autons);

        Auton* selected_auton;
        std::vector<Auton*> m_autons;

        void initialize();
        
        void setPathsFile(std::string);
        
        void timer_callback();

        void autonPeriodic();
    };
//}
