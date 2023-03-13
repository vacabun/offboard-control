#include "visualizer.hpp"
VisualizerNode::VisualizerNode() : Node("px4_visualizer")
{

    this->declare_parameter("subscribe_namespace", "/mavros/uas_1");
    this->declare_parameter("vehicle_name", "uas_1");

    subscribe_namespace = this->get_parameter("subscribe_namespace").get_parameter_value().get<std::string>();
    vehicle_name = this->get_parameter("vehicle_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "subscribe namespace : %s", subscribe_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "vehicle name : %s", vehicle_name.c_str());

    std::string velocity_topic_name = subscribe_namespace + "/local_position/velocity_local";
    std::string local_position_topic_name = subscribe_namespace + "/local_position/pose";
    std::string setpoint_topic_name = subscribe_namespace + "/setpoint_position/local";

    velocity_sub =
        this->create_subscription<geometry_msgs::msg::TwistStamped>(
            velocity_topic_name, 10, std::bind(&VisualizerNode::velocity_cb, this, std::placeholders::_1));
    local_position_sub =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            local_position_topic_name, 10, std::bind(&VisualizerNode::local_position_cb, this, std::placeholders::_1));
    setpoint_sub =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            setpoint_topic_name, 10, std::bind(&VisualizerNode::setpoint_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "attitude topic name : %s", velocity_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "local position topic name : %s", local_position_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "setpoint topic name : %s", setpoint_topic_name.c_str());

    std::string pose_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/pose";
    std::string path_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/path";
    std::string setpoint_path_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/setpoint_path";
    std::string velocity_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/velocity";

    pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_publisher_topic_name, 10);
    path_pub =
        this->create_publisher<nav_msgs::msg::Path>(path_publisher_topic_name, 10);
    setpoint_path_pub =
        this->create_publisher<nav_msgs::msg::Path>(setpoint_path_publisher_topic_name, 10);
    velocity_pub =
        this->create_publisher<visualization_msgs::msg::Marker>(velocity_publisher_topic_name, 10);

    double timer_period = 0.05;
    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds((int)(timer_period * 1000)),
            std::bind(&VisualizerNode::timer_callback, this));

    path_msg.header.frame_id = "map";
    setpoint_path_msg.header.frame_id = "map";
}

void VisualizerNode::timer_callback()
{
    pose_pub->publish(local_position);

    path_msg.poses.push_back(local_position);
    path_pub->publish(path_msg);

    setpoint_path_msg.poses.push_back(set_position);
    setpoint_path_pub->publish(setpoint_path_msg);

    visualization_msgs::msg::Marker velocity_msg;
    velocity_msg.action = visualization_msgs::msg::Marker::ADD;
    velocity_msg.header.frame_id = "map";
    velocity_msg.ns = "arrow";
    velocity_msg.id = 1;
    velocity_msg.type = visualization_msgs::msg::Marker::ARROW;
    velocity_msg.scale.x = 0.1;
    velocity_msg.scale.y = 0.2;
    velocity_msg.scale.z = 0.0;
    velocity_msg.color.r = 0.5;
    velocity_msg.color.g = 0.5;
    velocity_msg.color.b = 0.0;
    velocity_msg.color.a = 1.0;
    geometry_msgs::msg::Point tail_point;
    tail_point.x = local_position.pose.position.x;
    tail_point.y = local_position.pose.position.y;
    tail_point.z = local_position.pose.position.z;
    double dt = 0.3;
    geometry_msgs::msg::Point head_point;
    head_point.x = tail_point.x + dt * velocity.twist.linear.x;
    head_point.y = tail_point.y + dt * velocity.twist.linear.y;
    head_point.z = tail_point.z + dt * velocity.twist.linear.z;
    velocity_msg.points = {tail_point, head_point};
    velocity_pub->publish(velocity_msg);
}

void VisualizerNode::local_position_cb(const geometry_msgs::msg::PoseStamped msg)
{
    local_position = msg;
}
void VisualizerNode::setpoint_cb(const geometry_msgs::msg::PoseStamped msg)
{
    set_position = msg;
}
void VisualizerNode::velocity_cb(const geometry_msgs::msg::TwistStamped msg)
{
    velocity = msg;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();

    return 0;
}