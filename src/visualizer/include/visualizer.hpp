#ifndef _VISUALIZER_HPP_
#define _VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode();

private:
    void velocity_cb(const geometry_msgs::msg::TwistStamped msg);
    void local_position_cb(const geometry_msgs::msg::PoseStamped msg);
    void setpoint_cb(const geometry_msgs::msg::PoseStamped msg);
    void timer_callback();

    std::string subscribe_namespace;
    std::string vehicle_name;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub;

    geometry_msgs::msg::TwistStamped velocity;
    geometry_msgs::msg::PoseStamped local_position;
    geometry_msgs::msg::PoseStamped set_position;

    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path setpoint_path_msg;

};

#endif