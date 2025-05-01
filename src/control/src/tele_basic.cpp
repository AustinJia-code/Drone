/**
 * @file tele_basic.cpp
 * @brief Basic joystick controlled drone movement
 * 
 * TODO: Add functionality to switch to a different controller
 */

#include "tele_basic.hpp"

TeleBasic::TeleBasic (std::shared_ptr<PX4Controller> controller,
                      std::shared_ptr<rclcpp::Node> node)
  : TeleController (controller, node) {}

bool TeleBasic::loop ()
{
  // Check for disconnect
  if ((clock_.now () - last_joy_time_).seconds () > 1.0)
    return false;

  // Simple velocity control from joysticks
  controller_->publish_velocity_setpoint
  (
    joysticks_.left_x, joysticks_.left_y,
    joysticks_.right_x, joysticks_.right_y
  );

  return true;
}

void TeleBasic::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Store joysticks
  if (msg->axes.size() > 3)
  {
    joysticks_ = {msg->axes [0], msg->axes [1], msg->axes [2], msg->axes [3]};
    last_joy_time_ = clock_.now ();
  }
}
