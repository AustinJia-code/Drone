/**
 * @file tele_controller.hpp
 * @brief Structure for a teleop controller
 */

#pragma once

#include "px4_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * TeleController
 */
class TeleController
{
public:
  TeleController (std::shared_ptr<PX4Controller> controller,
                  std::shared_ptr<rclcpp::Node> node)
    : controller_ (controller), node_ (node) {}
  
  virtual ~TeleController () = default;

  virtual bool loop () = 0;

  virtual void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

protected:
  std::shared_ptr<PX4Controller> controller_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Time last_joy_time_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};