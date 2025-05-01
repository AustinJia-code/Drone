/**
 * @file tele_basic.hpp
 * @brief Basic joystick velocity control telop
 */

#pragma once

#include <memory>
#include "px4_controller.hpp"
#include "tele_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleBasic : public TeleController
{
public:
  explicit TeleBasic (std::shared_ptr<PX4Controller> controller,
                                 std::shared_ptr<rclcpp::Node> node)
    : TeleController (controller, node) {}
                      
  ~TeleBasic () override = default;

  bool control_loop () override
  {
    // Simple velocity control from joysticks
    controller_->publish_velocity_setpoint
    (
      joysticks_.left_x, joysticks_.left_y,
      joysticks_.right_x, joysticks_.right_y
    );

    return true;
  }

  void handle_joy (const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    // Store joysticks
    if (msg->axes.size() > 3)
      joysticks_ = {msg->axes [0], msg->axes [1], msg->axes [2], msg->axes [3]};
  }

private:
  /**
   * Joystick values
   */
  struct Joysticks { float left_x, left_y, right_x, right_y; };
  Joysticks joysticks_;
};
