/**
 * @file gamepad_bridge.cpp
 * @brief ROS node for gamepad inputs
 */

 #include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gamepad_node");
    RCLCPP_INFO(node->get_logger(), "Gamepad node running");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
