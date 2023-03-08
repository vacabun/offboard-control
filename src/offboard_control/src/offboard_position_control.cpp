// https://docs.px4.io/v1.12/en/ros/mavros_offboard.html
// https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/px4_offboard/offboard_control.py
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html

#include "offboard_position_control.hpp"
using namespace std::chrono_literals;
OffBoardPositionNode::OffBoardPositionNode() : Node("offboard_position_control")
{
    this->declare_parameter("subscribe_namespace", "/mavros/uas_1");
    subscribeNamespace = this->get_parameter("subscribe_namespace").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "subscribe namespace : %s", subscribeNamespace.c_str());

    std::string state_topic_name = subscribeNamespace + "/state";
    std::string local_pos_publisher_name = subscribeNamespace + "/setpoint_position/local";
    std::string arming_client_services_name = subscribeNamespace + "/cmd/arming";
    std::string set_mode_client_services_name = subscribeNamespace + "/set_mode";

    RCLCPP_INFO(this->get_logger(), "state topic name : %s", state_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "local position publisher name : %s", local_pos_publisher_name.c_str());
    RCLCPP_INFO(this->get_logger(), "arming client services name : %s", arming_client_services_name.c_str());
    RCLCPP_INFO(this->get_logger(), "set mode client services name : %s", set_mode_client_services_name.c_str());

    state_sub =
        this->create_subscription<mavros_msgs::msg::State>(
            state_topic_name, 10, std::bind(&OffBoardPositionNode::state_cb, this, std::placeholders::_1));

    local_pos_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(local_pos_publisher_name, 10);

    arming_client =
        this->create_client<mavros_msgs::srv::CommandBool>(arming_client_services_name);

    set_mode_client =
        this->create_client<mavros_msgs::srv::SetMode>(set_mode_client_services_name);

    bool state_is_connected = false;

    while (!state_is_connected)
    {
        rclcpp::WaitSet wait_set;
        wait_set.add_subscription(state_sub);
        if (wait_set.wait(5s).kind() == rclcpp::WaitResultKind::Ready)
        {
            mavros_msgs::msg::State msg;
            rclcpp::MessageInfo info;
            auto ret_take = state_sub->take(msg, info);
            if (ret_take)
            {
                state_is_connected = msg.connected;
                if (state_is_connected)
                    RCLCPP_INFO(this->get_logger(), "got message. state connected is ture.");
                else
                    RCLCPP_INFO(this->get_logger(), "got message. state connected is false.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "no message received");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "couldn't wait for message");
        }
    }

    while (!arming_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    while (!set_mode_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "Init finished.");

    offb_set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    arm_cmd_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();

    timer_ = this->create_wall_timer(1s, std::bind(&OffBoardPositionNode::timer_callback, this));
}
void OffBoardPositionNode::timer_callback()
{
    if (current_state.mode != "OFFBOARD")
    {
        offb_set_mode_request->custom_mode = "OFFBOARD";
        using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            if (future.get()->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Offboard enabled");
            }
        };
        auto future_result = set_mode_client->async_send_request(offb_set_mode_request, response_received_callback);
    }
    else if (!current_state.armed)
    {
        arm_cmd_request->value = true;
        using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future)
        {
            if (future.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
            }
        };
        auto future_result = arming_client->async_send_request(arm_cmd_request, response_received_callback);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
    local_pos_pub->publish(pose);
}
void OffBoardPositionNode::state_cb(const mavros_msgs::msg::State msg)
{
    RCLCPP_INFO(this->get_logger(), "get state msg.");
    current_state = msg;
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
