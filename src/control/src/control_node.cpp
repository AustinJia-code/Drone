/**
 * @file control_node.cpp
 * @brief ROS node for drone control
 * @cite https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp
 */

 #include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensor_node");
    RCLCPP_INFO(node->get_logger(), "Sensor node running");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
