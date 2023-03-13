#include "visualizer.hpp"
VisualizerNode::VisualizerNode() : Node("px4_visualizer")
{

    this->declare_parameter("subscribe_namespace", "/mavros/uas_1");
    this->declare_parameter("vehicle_name", "uas_1");

    subscribeNamespace = this->get_parameter("subscribe_namespace").get_parameter_value().get<std::string>();
    vehicle_name = this->get_parameter("vehicle_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "subscribe namespace : %s", subscribeNamespace.c_str());
    RCLCPP_INFO(this->get_logger(), "vehicle name : %s", vehicle_name.c_str());

    std::string odometry_topic_name = subscribeNamespace + "/altitude";
    std::string local_position_topic_name = subscribeNamespace + "/local_position/pose";
    std::string setpoint_topic_name = subscribeNamespace + "/setpoint_position/local";

    odometry_sub =
        this->create_subscription<nav_msgs::msg::Odometry>(
           odometry_topic_name, 10, std::bind(&VisualizerNode::odometry_cb, this, std::placeholders::_1));
    local_position_sub =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            local_position_topic_name, 10, std::bind(&VisualizerNode::local_position_cb, this, std::placeholders::_1));
    setpoint_sub =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            setpoint_topic_name, 10, std::bind(&VisualizerNode::setpoint_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "attitude topic name : %s", attitude_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "local position topic name : %s", local_position_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "setpoint topic name : %s", setpoint_topic_name.c_str());

    std::string pose_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/pose";
    std::string velocity_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/velocity";
    std::string path_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/path";
    std::string setpoint_path_publisher_topic_name = "/px4_visualizer/" + vehicle_name + "/setpoint_path";

    pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_publisher_topic_name, 10);
    velocity_pub =
        this->create_publisher<visualization_msgs::msg::Marker>(velocity_publisher_topic_name, 10);
    path_pub =
        this->create_publisher<nav_msgs::msg::Path>(path_publisher_topic_name, 10);
    setpoint_path_pub =
        this->create_publisher<nav_msgs::msg::Path>(setpoint_path_publisher_topic_name, 10);

    double timer_period = 0.05;
    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds((int)(timer_period * 1000)),
            std::bind(&VisualizerNode::timer_callback, this));
}

void VisualizerNode::timer_callback()
{
    geometry_msgs::msg::PoseStamped vehicle_pose_msg;
    vehicle_pose_msg =
        vector2PoseMsg('map', self.vehicle_local_position, self.vehicle_attitude);

    pose_pub->publish(vehicle_pose_msg);
}
void VisualizerNode::odometry_cb(const nav_msgs::msg::Odometry msg)
{

}
void VisualizerNode::local_position_cb(const geometry_msgs::msg::PoseStamped msg)
{
}
void VisualizerNode::setpoint_cb(const geometry_msgs::msg::PoseStamped msg)
{
}
visualization_msgs::msg::Marker VisualizerNode::create_arrow_marker(int32_t id)
{
    visualization_msgs::msg::Marker msg;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.header.frame_id = "map";
    msg.ns = "arrow";
    msg.id = id;
    msg.type = visualization_msgs::msg::Marker::ARROW;
    msg.scale.x = 0.1;
    msg.scale.y = 0.2;
    msg.scale.z = 0.0;
    msg.color.r = 0.5;
    msg.color.g = 0.5;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    double dt = 0.3;
    geometry_msgs::msg::Point tail_point;
    tail_point.x = tail[0];
    tail_point.y = tail[1];
    tail_point.z = tail[2];

    geometry_msgs::msg::Point head_point;
    head_point.x = tail[0] + dt * vector[0];
    head_point.y = tail[1] + dt * vector[1];
    head_point.z = tail[2] + dt * vector[2];
    msg.points = {tail_point, head_point};
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