// https://docs.px4.io/v1.12/en/ros/mavros_offboard.html
// https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/px4_offboard/offboard_control.py
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html

#include "offboard_position_control.hpp"

OffBoardPositionNode::OffBoardPositionNode() : Node("offboard_position_control")
{
    this->declare_parameter("subscribe_namespace", "/mavros/uas_1");
    subscribeNamespace = this->get_parameter("subscribe_namespace").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "subscribe namespace : %s", subscribeNamespace.c_str());

    state_sub =
        this->create_subscription<mavros_msgs::msg::State>(
            subscribeNamespace + "/state", 10, std::bind(
                &OffBoardPositionNode::state_cb, this, std::placeholders::_1));
    
    local_pos_pub = 
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            subscribeNamespace + "/setpoint_position/local", 10);

    arming_client =
        this->create_client<mavros_msgs::srv::CommandBool>(subscribeNamespace + "/cmd/arming");

    set_mode_client =
        this->create_client<mavros_msgs::srv::SetMode>(subscribeNamespace + "/set_mode");
    
}
void OffBoardPositionNode::state_cb(const mavros_msgs::msg::State::ConstPtr &msg)
{
    current_state = *msg;
    msg->
}
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffBoardPositionNode>());
    rclcpp::shutdown();

    return 0;
}
