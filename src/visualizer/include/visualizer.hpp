#ifndef _VISUALIZER_HPP_
#define _VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <mavros_msgs/msg/altitude.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode();

private:
    void VisualizerNode::odometry_cb(const nav_msgs::msg::Odometry msg);
    void VisualizerNode::local_position_cb(const geometry_msgs::msg::PoseStamped msg);
    void VisualizerNode::setpoint_cb(const geometry_msgs::msg::PoseStamped msg);
    void VisualizerNode::timer_callback();
    visualization_msgs::msg::Marker VisualizerNode::create_arrow_marker();

    std::string subscribeNamespace;
    std::string vehicle_name;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub;
};

#endif