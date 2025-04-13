/**
 * @file camera_bridge.cpp
 * @brief ROS node for camera data
 * @cite https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/vehicle_gps_position_listener.cpp
 */

 #include "rclcpp/rclcpp.hpp"

 int main(int argc, char * argv[])
 {
   rclcpp::init(argc, argv);

   auto node = std::make_shared<rclcpp::Node>("camera_bridge");
   RCLCPP_INFO(node->get_logger(), "Sensor node running");

   rclcpp::spin(node);
   rclcpp::shutdown();
   
   return 0;
 }
 