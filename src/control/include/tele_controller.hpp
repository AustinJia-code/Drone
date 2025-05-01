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
   * Actual control loop
   */
  virtual bool control_loop () = 0;

  /**
   * Actual input handler
   */
  virtual void handle_joy (const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

  /**
   * Control loop wrapper (called by driver)
   */
  bool loop ()
  {
    // Check for disconnect
    if ((clock_.now () - last_joy_time_).seconds () > 1.0)
      return false;

    return control_loop ();
  }

  /**
   * Input handler wrapper
   */
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size () > 0 && msg->buttons.size () > 0)
      last_joy_time_ = clock_.now ();

    handle_joy (msg);
  }

protected:
  std::shared_ptr<PX4Controller> controller_;         // PX4 interface
  std::shared_ptr<rclcpp::Node> node_;                // Node for pub/subs

private:
  rclcpp::Clock clock_ {RCL_SYSTEM_TIME};             // Clock
  rclcpp::Time last_joy_time_;                        // Time of last user input
};