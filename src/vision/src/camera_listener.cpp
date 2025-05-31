/**
 * @file camera_listener.cpp
 * @brief ROS node for camera data
 */

 #include "rclcpp/rclcpp.hpp"

 int main (int argc, char * argv [])
 {
   rclcpp::init(argc, argv);

   auto node = std::make_shared<rclcpp::Node> ("camera_listener");
   RCLCPP_INFO (node->get_logger (), "Camera listener running");

   rclcpp::spin (node);
   rclcpp::shutdown ();
   
   return 0;
 }
 