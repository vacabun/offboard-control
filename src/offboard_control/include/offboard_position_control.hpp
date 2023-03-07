#ifndef _UWB_SIMULATION_
#define _UWB_SIMULATION_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OffBoardPositionNode : public rclcpp::Node
{
public:
    OffBoardPositionNode();

private:
    void OffBoardPositionNode::state_cb(const mavros_msgs::msg::State::ConstPtr &msg);
    std::string subscribeNamespace;
    mavros_msgs::msg::State current_state;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
};

#endif