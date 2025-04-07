/**
 * @file sensor_node.cpp
 * @brief ROS node for drone sensors
 * @cite https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/sensor_combined_listener.cpp
 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("imu_listener");
  RCLCPP_INFO(node->get_logger(), "Sensor node running");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
