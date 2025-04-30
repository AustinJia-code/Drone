/**
 * @file auto_controller.hpp
 * @brief Structure for an auto controller
 */

#pragma once

#include "px4_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * Auto controller
 */
class AutoController
{
public:
  AutoController (std::shared_ptr<PX4Controller> controller,
                  std::shared_ptr<rclcpp::Node> node)
    : controller_ (controller), node_ (node) {}
  
  virtual ~AutoController () = default;

  virtual void init () = 0;
  virtual bool loop () = 0;
  virtual bool is_over () = 0;

protected:
  std::shared_ptr<PX4Controller> controller_;
  std::shared_ptr<rclcpp::Node> node_;
  bool is_over_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  virtual void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
};