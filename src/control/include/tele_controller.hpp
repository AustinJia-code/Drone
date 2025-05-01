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
  /**
   * Constructor
   */
  TeleController (std::shared_ptr<PX4Controller> controller,
                  std::shared_ptr<rclcpp::Node> node)
    : controller_ (controller), node_ (node) {}
  
  /**
   * Destructor
   */
  virtual ~TeleController () = default;

  /**
   * Control loop
   */
  virtual bool loop () = 0;

  /**
   * Handles user inputs
   */
  virtual void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

protected:
  std::shared_ptr<PX4Controller> controller_;         // PX4 interface
  std::shared_ptr<rclcpp::Node> node_;                // Node for pub/subs

  rclcpp::Clock clock_ {RCL_SYSTEM_TIME};             // Clock
  rclcpp::Time last_joy_time_;                        // Time of last user input
};